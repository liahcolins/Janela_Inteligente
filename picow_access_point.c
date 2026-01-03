#include <string.h>
#include <stdio.h>
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "dhcpserver.h"
#include "dnsserver.h"

// --- Configuração dos Pinos do Motor de Passo (ULN2003) ---
// Usando pinos 2, 3, 4, 5 para não conflitar com os sensores nos pinos 16, 17, 18
#define IN1_PIN 19
#define IN2_PIN 20
#define IN3_PIN 4
#define IN4_PIN 9

// --- Configuração dos Sensores ---
#define RAIN_SENSOR_DO_PIN 16
#define PIR_OUT_PIN 17
#define TCRT_DO_PIN 18 

// --- Configurações do Motor ---
// Quantas voltas completas o motor deve dar para abrir a janela totalmente?
// Ajuste este valor conforme o tamanho da sua janela/mecanismo.
#define REVOLUCOES_ABERTURA 2 

// --- Configurações do Servidor Web ---
#define TCP_PORT 80
#define DEBUG_printf printf
#define POLL_TIME_S 5
#define HTTP_GET "GET"
#define HTTP_RESPONSE_HEADERS "HTTP/1.1 %d OK\nContent-Length: %d\nContent-Type: text/html; charset=utf-8\nConnection: close\n\n"
#define HTTP_RESPONSE_REDIRECT "HTTP/1.1 302 Redirect\nLocation: http://%s" WINDOW_CONTROL_URL "\n\n"
#define WINDOW_CONTROL_URL "/control"
#define WINDOW_PARAM "janela=%d"

// HTML atualizado
#define WINDOW_CONTROL_BODY "<html><head><title>Controle de Janela</title><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0;\"></head><body style=\"font-family: sans-serif; text-align: center;\"><h1>Controle da Janela</h1><p>Estado: <strong>%s</strong>.</p><p>%s</p><p>%s</p><p>%s</p><p style=\"margin-top: 2rem;\"><a href=\"?janela=%d\" style=\"padding: 1rem 2rem; background-color: %s; color: white; text-decoration: none; border-radius: 5px;\">%s</a></p></body></html>"

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    bool complete;
    ip_addr_t gw;
} TCP_SERVER_T;

typedef struct TCP_CONNECT_STATE_T_ {
    struct tcp_pcb *pcb;
    int sent_len;
    char headers[128];
    char result[2048];
    int header_len;
    int result_len;
    ip_addr_t *gw;
} TCP_CONNECT_STATE_T;

// Variável global para rastrear o estado da janela
// true = Aberta, false = Fechada
static bool window_is_open = false; 

// Flags globais dos sensores
static bool is_raining = false;
static bool motion_detected = false;
static bool obstacle_detected = false;

// --- LÓGICA DO MOTOR DE PASSO ---

const int half_step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

void move_step(int step_index) {
    gpio_put(IN1_PIN, half_step_sequence[step_index][0]);
    gpio_put(IN2_PIN, half_step_sequence[step_index][1]);
    gpio_put(IN3_PIN, half_step_sequence[step_index][2]);
    gpio_put(IN4_PIN, half_step_sequence[step_index][3]);
}

void stop_motor() {
    gpio_put(IN1_PIN, 0);
    gpio_put(IN2_PIN, 0);
    gpio_put(IN3_PIN, 0);
    gpio_put(IN4_PIN, 0);
}

// Função para mover a janela
// abrindo = true (Abre), abrindo = false (Fecha)
void move_window(bool abrindo) {
    // Se já estiver no estado desejado, não faz nada
    if (window_is_open == abrindo) {
        return;
    }

    const int steps_per_revolution = 4096;
    int total_steps = REVOLUCOES_ABERTURA * steps_per_revolution;
    int current_step_index = 0;
    
    printf("Acionando motor: %s a janela...\n", abrindo ? "ABRINDO" : "FECHANDO");

    for (int i = 0; i < total_steps; i++) {
        // --- VERIFICAÇÃO DE SEGURANÇA EM TEMPO REAL ---
        // Se estiver FECHANDO e detectar obstáculo/movimento, PARA TUDO.
        if (!abrindo) { // Se está fechando
             // Atualiza leitura rápida
             bool safety_trigger = gpio_get(PIR_OUT_PIN) || !gpio_get(TCRT_DO_PIN);
             if (safety_trigger) {
                 printf("PARADA DE EMERGENCIA: Obstaculo detectado durante fechamento!\n");
                 stop_motor();
                 // Mantém o estado como estava (aberta), pois não concluiu o fechamento
                 window_is_open = true; 
                 return; 
             }
        }

        // --- MANTER WI-FI VIVO ---
        // O motor de passo é lento. Precisamos processar o Wi-Fi durante o movimento
        // senão a conexão cai.
        #if PICO_CYW43_ARCH_POLL
        cyw43_arch_poll();
        #endif

        // Lógica de direção: Abrir (Clockwise) ou Fechar (Counter-Clockwise)
        // Se o seu motor girar ao contrário do esperado, troque 'abrindo' por '!abrindo' aqui
        if (abrindo) { 
            current_step_index = (current_step_index + 1) % 8;
        } else {
            current_step_index = (current_step_index - 1 + 8) % 8;
        }

        move_step(current_step_index);
        sleep_us(1000); // Velocidade do motor
    }
    
    stop_motor(); // Desliga bobinas para não esquentar
    window_is_open = abrindo; // Atualiza o estado oficial
    printf("Janela %s com sucesso.\n", abrindo ? "ABERTA" : "FECHADA");
}

// --- FIM DA LÓGICA DO MOTOR ---

static err_t tcp_close_client_connection(TCP_CONNECT_STATE_T *con_state, struct tcp_pcb *client_pcb, err_t close_err) {
    if (client_pcb) {
        assert(con_state && con_state->pcb == client_pcb);
        tcp_arg(client_pcb, NULL);
        tcp_poll(client_pcb, NULL, 0);
        tcp_sent(client_pcb, NULL);
        tcp_recv(client_pcb, NULL);
        tcp_err(client_pcb, NULL);
        err_t err = tcp_close(client_pcb);
        if (err != ERR_OK) {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(client_pcb);
            close_err = ERR_ABRT;
        }
        if (con_state) {
            free(con_state);
        }
    }
    return close_err;
}

static void tcp_server_close(TCP_SERVER_T *state) {
    if (state->server_pcb) {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *pcb, u16_t len) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    con_state->sent_len += len;
    if (con_state->sent_len >= con_state->header_len + con_state->result_len) {
        return tcp_close_client_connection(con_state, pcb, ERR_OK);
    }
    return ERR_OK;
}

static int server_content_handler(const char *request, const char *params, char *result, size_t max_result_len) {
    int len = 0;
    char rain_status_msg[128] = "";
    char motion_status_msg[128] = "";
    char obstacle_status_msg[128] = "";

    // Atualiza mensagens de status
    if (is_raining) snprintf(rain_status_msg, sizeof(rain_status_msg), "<p style=\"color: red;\">Chuva detectada!</p>");
    else snprintf(rain_status_msg, sizeof(rain_status_msg), "<p style=\"color: green;\">Tempo seco.</p>");

    if (motion_detected) snprintf(motion_status_msg, sizeof(motion_status_msg), "<p style=\"color: orange;\">MOVIMENTO DETECTADO!</p>");
    else snprintf(motion_status_msg, sizeof(motion_status_msg), "<p>Nenhum movimento.</p>");

    if (obstacle_detected) snprintf(obstacle_status_msg, sizeof(obstacle_status_msg), "<p style=\"color: orange;\">OBSTÁCULO DETECTADO!</p>");
    else snprintf(obstacle_status_msg, sizeof(obstacle_status_msg), "<p>Sem obstáculos.</p>");

    // Processa comandos manuais da Web
    if (strncmp(request, WINDOW_CONTROL_URL, strlen(WINDOW_CONTROL_URL)) == 0) {
        if (params) {
            int new_state_param;
            int items_scanned = sscanf(params, WINDOW_PARAM, &new_state_param);
            if (items_scanned == 1) {
                if (!is_raining) {
                    if (new_state_param == 1) { // Comando ABRIR
                        move_window(true);
                    } else { // Comando FECHAR
                        if (motion_detected || obstacle_detected) {
                            DEBUG_printf("Fechamento manual impedido: Segurança ativa.\n");
                        } else {
                            move_window(false);
                        }
                    }
                }
            }
        }
        
        // Gera HTML
        if (window_is_open) {
            len = snprintf(result, max_result_len, WINDOW_CONTROL_BODY, "ABERTA", rain_status_msg, motion_status_msg, obstacle_status_msg, 0, "#d9534f", "FECHAR JANELA");
        } else {
            len = snprintf(result, max_result_len, WINDOW_CONTROL_BODY, "FECHADA", rain_status_msg, motion_status_msg, obstacle_status_msg, 1, "#5cb85c", "ABRIR JANELA");
        }
    }
    return len;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    if (!p) {
        return tcp_close_client_connection(con_state, pcb, ERR_OK);
    }
    assert(con_state && con_state->pcb == pcb);
    if (p->tot_len > 0) {
        pbuf_copy_partial(p, con_state->headers, p->tot_len > sizeof(con_state->headers) - 1 ? sizeof(con_state->headers) - 1 : p->tot_len, 0);
        if (strncmp(HTTP_GET, con_state->headers, sizeof(HTTP_GET) - 1) == 0) {
            char *request = con_state->headers + sizeof(HTTP_GET);
            char *params = strchr(request, '?');
            if (params) {
                if (*params) {
                    char *space = strchr(request, ' ');
                    *params++ = 0;
                    if (space) *space = 0;
                } else { params = NULL; }
            } else {
                char *space = strchr(request, ' ');
                if (space) *space = 0;
            }
            con_state->result_len = server_content_handler(request, params, con_state->result, sizeof(con_state->result));
            if (con_state->result_len > 0) {
                con_state->header_len = snprintf(con_state->headers, sizeof(con_state->headers), HTTP_RESPONSE_HEADERS, 200, con_state->result_len);
            } else {
                con_state->header_len = snprintf(con_state->headers, sizeof(con_state->headers), HTTP_RESPONSE_REDIRECT, ipaddr_ntoa(con_state->gw));
            }
            tcp_write(pcb, con_state->headers, con_state->header_len, 0);
            if (con_state->result_len) {
                tcp_write(pcb, con_state->result, con_state->result_len, 0);
            }
        }
        tcp_recved(pcb, p->tot_len);
    }
    pbuf_free(p);
    return ERR_OK;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *pcb) {
    return tcp_close_client_connection(arg, pcb, ERR_OK);
}

static void tcp_server_err(void *arg, err_t err) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    if (err != ERR_ABRT) {
        tcp_close_client_connection(con_state, con_state->pcb, err);
    }
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        return ERR_VAL;
    }
    TCP_CONNECT_STATE_T *con_state = calloc(1, sizeof(TCP_CONNECT_STATE_T));
    if (!con_state) {
        return ERR_MEM;
    }
    con_state->pcb = client_pcb;
    con_state->gw = &state->gw;
    tcp_arg(client_pcb, con_state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);
    return ERR_OK;
}

static bool tcp_server_open(void *arg, const char *ap_name) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) return false;
    err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT);
    if (err) return false;
    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        if (pcb) tcp_close(pcb);
        return false;
    }
    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);
    printf("Ponto de acesso '%s' criado.\n", ap_name);
    return true;
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Aguarda USB
    printf("--- Controle de Janela com Motor de Passo e ULN2003 ---\n");

    // --- INICIALIZAÇÃO DO MOTOR ---
    gpio_init(IN1_PIN); gpio_set_dir(IN1_PIN, GPIO_OUT);
    gpio_init(IN2_PIN); gpio_set_dir(IN2_PIN, GPIO_OUT);
    gpio_init(IN3_PIN); gpio_set_dir(IN3_PIN, GPIO_OUT);
    gpio_init(IN4_PIN); gpio_set_dir(IN4_PIN, GPIO_OUT);
    stop_motor(); // Garante motor desligado no inicio

    // --- INICIALIZAÇÃO DOS SENSORES ---
    gpio_init(RAIN_SENSOR_DO_PIN); gpio_set_dir(RAIN_SENSOR_DO_PIN, GPIO_IN);
    gpio_init(PIR_OUT_PIN); gpio_set_dir(PIR_OUT_PIN, GPIO_IN);
    gpio_init(TCRT_DO_PIN); gpio_set_dir(TCRT_DO_PIN, GPIO_IN);

    // Inicializa Wi-Fi
    TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
    if (!state || cyw43_arch_init()) return 1;
    const char *ap_name = "Controle_Janela";
    const char *password = "senha123";
    cyw43_arch_enable_ap_mode(ap_name, password, CYW43_AUTH_WPA2_AES_PSK);
    ip4_addr_t mask;
    IP_ADDR4(&state->gw, 192,168,4,1);
    IP_ADDR4(&mask, 255,255,255,0);
    dhcp_server_t dhcp_server;
    dhcp_server_init(&dhcp_server, &state->gw, &mask);
    dns_server_t dns_server;
    dns_server_init(&dns_server, &state->gw);
    if (!tcp_server_open(state, ap_name)) return 1;

    // Loop Principal
    while(true) {
        #if PICO_CYW43_ARCH_POLL
        cyw43_arch_poll();
        #endif

        // Leitura dos sensores
        is_raining = !gpio_get(RAIN_SENSOR_DO_PIN);
        motion_detected = gpio_get(PIR_OUT_PIN);
        obstacle_detected = !gpio_get(TCRT_DO_PIN);

        // Lógica de Automação
        if (window_is_open) {
            // Se estiver chovendo, tentar fechar
            if (is_raining) {
                if (motion_detected || obstacle_detected) {
                    // Não faz nada, espera limpar a área
                    printf("Chuva detectada, mas bloqueio de segurança ativo.\n");
                } else {
                    printf("Fechando automaticamente (chuva)...\n");
                    move_window(false); // Fecha
                }
            }
        }
        
        sleep_ms(100); // Intervalo de verificação
    }
    return 0;
}