#include <string.h>
#include <stdio.h>
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h" 
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "dhcpserver.h"
#include "dnsserver.h"

// ============================================================
// CONFIGURAÇÕES DE HARDWARE
// ============================================================

// --- Motor de Passo (ULN2003) ---
#define IN1_PIN 19
#define IN2_PIN 20
#define IN3_PIN 4
#define IN4_PIN 9
#define TOTAL_PASSOS 1366 // Aprox 120 graus

// --- Sensores de Segurança/Chuva ---
#define RAIN_SENSOR_DO_PIN 16
#define PIR_OUT_PIN 17
#define TCRT_DO_PIN 18 
#define CICLOS_FILTRO_MOVIMENTO 20 

// --- Sensor BMP280 (I2C) ---
#define I2C_PORT i2c1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
#define BMP280_ADDR 0x76

// ============================================================
// VARIÁVEIS GLOBAIS
// ============================================================

static bool window_is_open = false; 
static bool is_raining = false;
static bool motion_detected = false;
static bool obstacle_detected = false;

// Variável Global de Temperatura
static float current_temp_c = 0.0;

// Variáveis para "Latch" (Memória temporária HTML)
static uint64_t last_rain_time = 0;
static uint64_t last_motion_time = 0;
static uint64_t last_obstacle_time = 0;
static int pir_filter_counter = 0;

// Variáveis de Calibração do BMP280
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
int32_t t_fine;

// Configurações Web
#define TCP_PORT 80
#define POLL_TIME_S 5
#define HTTP_GET "GET"
#define HTTP_RESPONSE_HEADERS "HTTP/1.1 %d OK\nContent-Length: %d\nContent-Type: text/html; charset=utf-8\nConnection: close\n\n"
// Header de Redirecionamento (Limpa a URL)
#define HTTP_RESPONSE_REDIRECT "HTTP/1.1 302 Redirect\nLocation: http://%s" WINDOW_CONTROL_URL "\n\n"
#define WINDOW_CONTROL_URL "/control"
#define WINDOW_PARAM "janela=%d"

// HTML
#define WINDOW_CONTROL_BODY "<html><head><title>Janela Inteligente TCC</title><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><meta http-equiv=\"refresh\" content=\"3\"></head><body style=\"font-family: sans-serif; text-align: center;\"><h1>Painel TCC</h1><div style=\"border: 1px solid #ccc; padding: 10px; margin: 10px; border-radius: 10px;\"><h3>Ambiente</h3><p>Temperatura: <strong style=\"font-size: 1.5rem;\">%.1f &deg;C</strong></p></div><div style=\"border: 1px solid #ccc; padding: 10px; margin: 10px; border-radius: 10px;\"><h3>Status</h3><p>Janela: <strong>%s</strong></p><p>%s</p><p>%s</p><p>%s</p></div><p style=\"margin-top: 2rem;\"><a href=\"?janela=%d\" style=\"padding: 1rem 2rem; background-color: %s; color: white; text-decoration: none; border-radius: 5px;\">%s</a></p></body></html>"

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

// ============================================================
// DRIVER BMP280 
// ============================================================

void bmp_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, BMP280_ADDR, buf, 2, false);
}

void bmp_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    i2c_write_blocking(I2C_PORT, BMP280_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, BMP280_ADDR, buf, len, false);
}

uint16_t read_u16(uint8_t reg) {
    uint8_t buf[2];
    bmp_read(reg, buf, 2);
    return (buf[1] << 8) | buf[0];
}

int16_t read_s16(uint8_t reg) {
    return (int16_t)read_u16(reg);
}

void bmp280_read_calibration() {
    dig_T1 = read_u16(0x88);
    dig_T2 = read_s16(0x8A);
    dig_T3 = read_s16(0x8C);
}

void bmp280_init() {
    bmp_write(0xF4, 0x27); 
    bmp_write(0xF5, 0xA0); 
    bmp280_read_calibration();
}

int32_t bmp280_compensate_T(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

void update_environment_data() {
    uint8_t data[6];
    bmp_read(0xF7, data, 6);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    int32_t temp_int = bmp280_compensate_T(adc_T);
    current_temp_c = temp_int / 100.0;
}

// ============================================================
// MOTOR E LÓGICA
// ============================================================

const int half_step_sequence[8][4] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
    {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

void move_step(int step_index) {
    gpio_put(IN1_PIN, half_step_sequence[step_index][0]);
    gpio_put(IN2_PIN, half_step_sequence[step_index][1]);
    gpio_put(IN3_PIN, half_step_sequence[step_index][2]);
    gpio_put(IN4_PIN, half_step_sequence[step_index][3]);
}

void stop_motor() {
    gpio_put(IN1_PIN, 0); gpio_put(IN2_PIN, 0); gpio_put(IN3_PIN, 0); gpio_put(IN4_PIN, 0);
}

void move_window(bool abrindo) {
    if (window_is_open == abrindo) return;
    int steps_to_move = TOTAL_PASSOS;
    int current_step_index = 0;
    printf("\n--- MOTOR: %s ---\n", abrindo ? "ABRINDO" : "FECHANDO");

    for (int i = 0; i < steps_to_move; i++) {
        if (!abrindo) { 
             if (gpio_get(PIR_OUT_PIN) || !gpio_get(TCRT_DO_PIN)) {
                 printf("!!! EMERGENCIA: Objeto detectado! !!!\n");
                 stop_motor();
                 window_is_open = true; 
                 return; 
             }
        }
        #if PICO_CYW43_ARCH_POLL
        cyw43_arch_poll();
        #endif
        if (abrindo) current_step_index = (current_step_index + 1) % 8;
        else current_step_index = (current_step_index - 1 + 8) % 8;
        move_step(current_step_index);
        sleep_us(1000); 
    }
    stop_motor();
    window_is_open = abrindo; 
    printf("Status: Janela %s.\n", abrindo ? "ABERTA" : "FECHADA");
}

// ============================================================
// SERVIDOR WEB (A LÓGICA CORRIGIDA ESTÁ AQUI)
// ============================================================

static err_t tcp_close_client_connection(TCP_CONNECT_STATE_T *con_state, struct tcp_pcb *client_pcb, err_t close_err) {
    if (client_pcb) {
        tcp_arg(client_pcb, NULL); tcp_poll(client_pcb, NULL, 0); tcp_sent(client_pcb, NULL);
        tcp_recv(client_pcb, NULL); tcp_err(client_pcb, NULL); tcp_close(client_pcb);
        if (con_state) free(con_state);
    }
    return close_err;
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *pcb, u16_t len) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    con_state->sent_len += len;
    if (con_state->sent_len >= con_state->header_len + con_state->result_len) return tcp_close_client_connection(con_state, pcb, ERR_OK);
    return ERR_OK;
}

// CORREÇÃO CRÍTICA AQUI:
static int server_content_handler(const char *request, const char *params, char *result, size_t max_result_len) {
    int len = 0;

    // Se existirem parâmetros (Ex: ?janela=1), processamos o comando MAS NÃO geramos HTML.
    // Retornamos len = 0. Isso fará o código principal enviar um REDIRECT (302).
    if (strncmp(request, WINDOW_CONTROL_URL, strlen(WINDOW_CONTROL_URL)) == 0) {
        if (params) {
            int new_state;
            if (sscanf(params, WINDOW_PARAM, &new_state) == 1) {
                if (!is_raining) {
                    if (new_state == 1) move_window(true);
                    else {
                        if (motion_detected || obstacle_detected) printf("WEB: Fechamento Manual Bloqueado por Seguranca.\n");
                        else move_window(false);
                    }
                }
            }
            // TRUQUE: Retornamos 0 para forçar o redirecionamento e limpar a URL
            return 0; 
        }
        
        // Se NÃO houver parâmetros (apenas visualização), geramos o HTML normal.
        char rain_msg[128] = "", motion_msg[128] = "", obst_msg[128] = "";

        if (is_raining) snprintf(rain_msg, sizeof(rain_msg), "<p style=\"color: red;\">Chuva detectada!</p>");
        else snprintf(rain_msg, sizeof(rain_msg), "<p style=\"color: green;\">Tempo seco.</p>");

        if (motion_detected) snprintf(motion_msg, sizeof(motion_msg), "<p style=\"color: orange;\">Movimento recente!</p>");
        else snprintf(motion_msg, sizeof(motion_msg), "<p>Nenhum movimento.</p>");

        if (obstacle_detected) snprintf(obst_msg, sizeof(obst_msg), "<p style=\"color: orange;\">Obstaculo recente!</p>");
        else snprintf(obst_msg, sizeof(obst_msg), "<p>Sem obstaculos.</p>");

        len = snprintf(result, max_result_len, WINDOW_CONTROL_BODY, 
             current_temp_c, 
             window_is_open ? "ABERTA" : "FECHADA", 
             rain_msg, motion_msg, obst_msg, 
             window_is_open ? 0 : 1, 
             window_is_open ? "#d9534f" : "#5cb85c", 
             window_is_open ? "FECHAR" : "ABRIR");
    }
    return len;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    if (!p) return tcp_close_client_connection(con_state, pcb, ERR_OK);
    if (p->tot_len > 0) {
        pbuf_copy_partial(p, con_state->headers, p->tot_len > sizeof(con_state->headers) - 1 ? sizeof(con_state->headers) - 1 : p->tot_len, 0);
        if (strncmp(HTTP_GET, con_state->headers, sizeof(HTTP_GET) - 1) == 0) {
            char *request = con_state->headers + sizeof(HTTP_GET); char *params = strchr(request, '?');
            if (params) { if (*params) { char *space = strchr(request, ' '); *params++ = 0; if (space) *space = 0; } else params = NULL; } else { char *space = strchr(request, ' '); if (space) *space = 0; }
            
            con_state->result_len = server_content_handler(request, params, con_state->result, sizeof(con_state->result));
            
            // LÓGICA DE DECISÃO: HTML vs REDIRECT
            if (con_state->result_len > 0) {
                // Envia Página HTML (200 OK)
                con_state->header_len = snprintf(con_state->headers, sizeof(con_state->headers), HTTP_RESPONSE_HEADERS, 200, con_state->result_len);
            } else {
                // Envia Redirecionamento (302) -> Limpa a URL!
                con_state->header_len = snprintf(con_state->headers, sizeof(con_state->headers), HTTP_RESPONSE_REDIRECT, ipaddr_ntoa(con_state->gw));
            }
            
            tcp_write(pcb, con_state->headers, con_state->header_len, 0);
            if (con_state->result_len) tcp_write(pcb, con_state->result, con_state->result_len, 0);
        }
        tcp_recved(pcb, p->tot_len);
    }
    pbuf_free(p);
    return ERR_OK;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *pcb) { return tcp_close_client_connection(arg, pcb, ERR_OK); }
static void tcp_server_err(void *arg, err_t err) { if (err != ERR_ABRT) tcp_close_client_connection(arg, ((TCP_CONNECT_STATE_T*)arg)->pcb, err); }

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) return ERR_VAL;
    TCP_CONNECT_STATE_T *con_state = calloc(1, sizeof(TCP_CONNECT_STATE_T));
    if (!con_state) return ERR_MEM;
    con_state->pcb = client_pcb; con_state->gw = &state->gw;
    tcp_arg(client_pcb, con_state); tcp_sent(client_pcb, tcp_server_sent); tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2); tcp_err(client_pcb, tcp_server_err);
    return ERR_OK;
}

static bool tcp_server_open(void *arg, const char *ap_name) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) return false;
    err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT);
    if (err) return false;
    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) { if (pcb) tcp_close(pcb); return false; }
    tcp_arg(state->server_pcb, state); tcp_accept(state->server_pcb, tcp_server_accept);
    printf("Ponto de acesso '%s' criado.\n", ap_name);
    return true;
}

// ============================================================
// MAIN LOOP
// ============================================================

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    printf("--- TCC Janela Inteligente: Iniciando ---\n");

    // Inicialização
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN); gpio_pull_up(I2C_SCL_PIN);
    bmp280_init(); 

    gpio_init(IN1_PIN); gpio_set_dir(IN1_PIN, GPIO_OUT);
    gpio_init(IN2_PIN); gpio_set_dir(IN2_PIN, GPIO_OUT);
    gpio_init(IN3_PIN); gpio_set_dir(IN3_PIN, GPIO_OUT);
    gpio_init(IN4_PIN); gpio_set_dir(IN4_PIN, GPIO_OUT);
    stop_motor(); 
    gpio_init(RAIN_SENSOR_DO_PIN); gpio_set_dir(RAIN_SENSOR_DO_PIN, GPIO_IN);
    gpio_init(PIR_OUT_PIN); gpio_set_dir(PIR_OUT_PIN, GPIO_IN);
    gpio_init(TCRT_DO_PIN); gpio_set_dir(TCRT_DO_PIN, GPIO_IN);

    TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
    if (!state || cyw43_arch_init()) return 1;
    cyw43_arch_enable_ap_mode("Controle_Janela", "senha123", CYW43_AUTH_WPA2_AES_PSK);
    ip4_addr_t mask; IP_ADDR4(&state->gw, 192,168,4,1); IP_ADDR4(&mask, 255,255,255,0);
    dhcp_server_t dhcp_server; dhcp_server_init(&dhcp_server, &state->gw, &mask);
    dns_server_t dns_server; dns_server_init(&dns_server, &state->gw);
    if (!tcp_server_open(state, "Controle_Janela")) return 1;

    window_is_open = false; 
    uint64_t last_print_time = 0;
    uint64_t last_temp_read_time = 0; 
    uint64_t last_block_alert_time = 0; 

    bool prev_rain_raw = false;
    bool prev_motion_confirmed = false;
    bool prev_obst_raw = false;

    while(true) {
        #if PICO_CYW43_ARCH_POLL
        cyw43_arch_poll();
        #endif

        bool rain_raw = !gpio_get(RAIN_SENSOR_DO_PIN);
        bool motion_raw = gpio_get(PIR_OUT_PIN);
        bool obst_raw = !gpio_get(TCRT_DO_PIN);

        // Filtro Movimento
        bool motion_confirmed_now = false;
        if (motion_raw) {
            pir_filter_counter++;
            if (pir_filter_counter >= CICLOS_FILTRO_MOVIMENTO) {
                motion_confirmed_now = true;
                pir_filter_counter = CICLOS_FILTRO_MOVIMENTO; 
            }
        } else {
            pir_filter_counter = 0;
            motion_confirmed_now = false;
        }

        if (rain_raw) last_rain_time = time_us_64();
        if (motion_confirmed_now) last_motion_time = time_us_64();
        if (obst_raw) last_obstacle_time = time_us_64();

        is_raining = (time_us_64() - last_rain_time < 3000000);
        motion_detected = (time_us_64() - last_motion_time < 3000000);
        obstacle_detected = (time_us_64() - last_obstacle_time < 3000000);

        if (time_us_64() - last_temp_read_time > 2000000) {
            update_environment_data(); 
            last_temp_read_time = time_us_64();
        }

        if (rain_raw && !prev_rain_raw) printf(">>> ALERTA: Chuva INICIOU!\n");
        if (motion_confirmed_now && !prev_motion_confirmed) printf(">>> ALERTA: Movimento DETECTADO!\n");
        if (obst_raw && !prev_obst_raw) printf(">>> ALERTA: Obstaculo DETECTADO!\n");

        prev_rain_raw = rain_raw;
        prev_motion_confirmed = motion_confirmed_now;
        prev_obst_raw = obst_raw;

        if (time_us_64() - last_print_time > 1000000) {
            printf("Resumo: Temp:%.1fC | Chuva:%s | Mov:%s | Obst:%s | Janela:%s\n",
                current_temp_c,
                is_raining ? "SIM" : "NAO",
                motion_detected ? "SIM" : "NAO",
                obstacle_detected ? "SIM" : "NAO",
                window_is_open ? "ABERTA" : "FECHADA");
            last_print_time = time_us_64();
        }

        // --- AUTOMAÇÃO TCC ---
        // Configurado para: Abre > 28C, Fecha < 25C
        
        if (is_raining) {
            if (window_is_open) {
                if (motion_detected || obstacle_detected) {
                    if (time_us_64() - last_block_alert_time > 500000) {
                        printf("ALERTA DE BLOQUEIO: Chuva detectada, mas fechamento impedido por Obstaculo/Movimento!\n");
                        last_block_alert_time = time_us_64();
                    }
                } else {
                    printf("AUTO: Chuva detectada. Fechando...\n");
                    move_window(false); 
                }
            }
        } 
        else {
            if (current_temp_c > 28.0 && !window_is_open) {
                 printf("AUTO: Calor (%.1f C). Abrindo...\n", current_temp_c);
                 move_window(true);
            }
            else if (current_temp_c < 23.0 && window_is_open) {
                if (motion_detected || obstacle_detected) {
                    if (time_us_64() - last_block_alert_time > 500000) {
                        printf("ALERTA DE BLOQUEIO: Tentativa de fechar (Frio) impedida por Obstaculo/Movimento!\n");
                        last_block_alert_time = time_us_64();
                    }
                } else {
                    printf("AUTO: Frio (%.1f C). Fechando...\n", current_temp_c);
                    move_window(false);
                }
            }
        }
        
        sleep_ms(10);
    }
    return 0;
}