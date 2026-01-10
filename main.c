#include "configura_geral.h" 
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/apps/mqtt.h"
#include "hardware/i2c.h"
#include <string.h> 

// ============================================================
// VARIÁVEIS GLOBAIS
// ============================================================
static bool window_is_open = false; 
static bool is_raining = false;
static bool motion_detected = false;
static bool obstacle_detected = false;
static bool prev_motion = false;
static bool prev_obstacle = false;
static bool prev_rain = false;
static bool override_ativo = false;       
static uint64_t fim_do_override = 0;       
static float current_temp_c = 0.0;
static uint64_t last_pub_time = 0;
static uint64_t last_temp_read_time = 0;
static int pir_filter_counter = 0;
static mqtt_client_t* mqtt_client;
static bool mqtt_connected = false;

uint16_t dig_T1; int16_t dig_T2, dig_T3; int32_t t_fine;

// ============================================================
// DRIVERS E FUNÇÕES AUXILIARES
// ============================================================
void bmp_write(uint8_t reg, uint8_t val) { uint8_t buf[2] = {reg, val}; i2c_write_blocking(I2C_PORT, BMP280_ADDR, buf, 2, false); }
void bmp_read(uint8_t reg, uint8_t *buf, uint8_t len) { i2c_write_blocking(I2C_PORT, BMP280_ADDR, &reg, 1, true); i2c_read_blocking(I2C_PORT, BMP280_ADDR, buf, len, false); }
uint16_t read_u16(uint8_t reg) { uint8_t buf[2]; bmp_read(reg, buf, 2); return (buf[1] << 8) | buf[0]; }
int16_t read_s16(uint8_t reg) { return (int16_t)read_u16(reg); }
void bmp280_init() { bmp_write(0xF4, 0x27); bmp_write(0xF5, 0xA0); dig_T1 = read_u16(0x88); dig_T2 = read_s16(0x8A); dig_T3 = read_s16(0x8C); }
int32_t bmp280_compensate_T(int32_t adc_T) { int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11; int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14; t_fine = var1 + var2; return (t_fine * 5 + 128) >> 8; }
void update_environment_data() { uint8_t data[6]; bmp_read(0xF7, data, 6); int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4); current_temp_c = bmp280_compensate_T(adc_T) / 100.0; }

const int half_step_sequence[8][4] = {{1,0,0,0},{1,1,0,0},{0,1,0,0},{0,1,1,0},{0,0,1,0},{0,0,1,1},{0,0,0,1},{1,0,0,1}};
void stop_motor() { gpio_put(IN1_PIN, 0); gpio_put(IN2_PIN, 0); gpio_put(IN3_PIN, 0); gpio_put(IN4_PIN, 0); }
void move_step(int step_index) { gpio_put(IN1_PIN, half_step_sequence[step_index][0]); gpio_put(IN2_PIN, half_step_sequence[step_index][1]); gpio_put(IN3_PIN, half_step_sequence[step_index][2]); gpio_put(IN4_PIN, half_step_sequence[step_index][3]); }

void mqtt_pub_request_cb(void *arg, err_t result) { if(result != ERR_OK) printf("MQTT: Erro pub (%d)\n", result); }

void publicar(const char* topico, const char* valor) {
    if (mqtt_connected) mqtt_publish(mqtt_client, topico, valor, strlen(valor), 0, 0, mqtt_pub_request_cb, NULL);
}

// --- FUNÇÃO DE MOVIMENTO (COM SEGURANÇA E MOTIVOS) ---
void move_window(bool abrindo, const char* motivo) {
    if (window_is_open == abrindo) return;
    
    // 1. Avisa que vai começar
    publicar(TOPICO_MOTIVO_DETALHE, motivo);
    printf("\n>>> ACAO: %s | MOTIVO: %s <<<\n", abrindo ? "ABRINDO" : "FECHANDO", motivo);

    int steps = TOTAL_PASSOS; int idx = 0;
    
    for (int i = 0; i < steps; i++) {
        // --- SEGURANÇA CRÍTICA ---
        if (!abrindo) { 
             bool seguranca_movimento = gpio_get(PIR_OUT_PIN);
             bool seguranca_obstaculo = !gpio_get(TCRT_DO_PIN);

             if (seguranca_movimento || seguranca_obstaculo) {
                 printf("[ALERTA] PARADA DE EMERGENCIA!\n");
                 stop_motor(); 
                 window_is_open = true; 
                 
                 publicar(TOPICO_STATUS_SIMPLES, "ABERTA"); 
                 publicar(TOPICO_MOTIVO_DETALHE, "BLOQUEIO: Objeto ou Movimento!"); 
                 return; 
             }
        }
        #if PICO_CYW43_ARCH_POLL
        cyw43_arch_poll();
        #endif
        idx = abrindo ? (idx + 1) % 8 : (idx - 1 + 8) % 8;
        move_step(idx); sleep_us(1000); 
    }
    stop_motor(); 
    window_is_open = abrindo; 
    
    // 2. Avisa que terminou
    publicar(TOPICO_STATUS_SIMPLES, abrindo ? "ABERTA" : "FECHADA");
    publicar(TOPICO_MOTIVO_DETALHE, "Operacao concluida com sucesso");
}

// ============================================================
// MQTT CALLBACKS
// ============================================================
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    char payload[32];
    if (len >= sizeof(payload)) len = sizeof(payload) - 1;
    memcpy(payload, data, len); payload[len] = '\0';
    printf("\n[MQTT COMANDO] Recebido: %s\n", payload);

    if (strcmp(payload, "abrir") == 0) {
        printf("Timer Acionado.\n");
        override_ativo = true;
        fim_do_override = to_us_since_boot(get_absolute_time()) + (TEMPO_OVERRIDE_MS * 1000);
        publicar(TOPICO_PUBLICA_MODO, "MANUAL_TEMP");
        move_window(true, "Comando Manual (App)");
        publicar(TOPICO_MOTIVO_DETALHE, "Timer Acionado (Aguardando...)");
    } 
    else if (strcmp(payload, "fechar") == 0) {
        printf("Timer Acionado.\n");
        override_ativo = true;
        fim_do_override = to_us_since_boot(get_absolute_time()) + (TEMPO_OVERRIDE_MS * 1000);
        publicar(TOPICO_PUBLICA_MODO, "MANUAL_TEMP");
        move_window(false, "Comando Manual (App)");
        publicar(TOPICO_MOTIVO_DETALHE, "Timer Acionado (Aguardando...)");
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT: Conectado!\n");
        mqtt_connected = true;
        mqtt_sub_unsub(client, TOPICO_SUBSCREVE_CMD, 0, mqtt_pub_request_cb, NULL, 1);
    } else {
        printf("MQTT: Falha conexao (%d)\n", status);
        mqtt_connected = false;
    }
}

void mqtt_do_connect() {
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico_w_janela";
    ci.keep_alive = 60;
    ip_addr_t broker_ip;
    ip4addr_aton(MQTT_BROKER_IP, &broker_ip);
    mqtt_client_connect(mqtt_client, &broker_ip, MQTT_BROKER_PORT, mqtt_connection_cb, NULL, &ci);
}

// ============================================================
// MAIN LOOP
// ============================================================
int main() {
    stdio_init_all();
    sleep_ms(2000); 

    i2c_init(I2C_PORT, 100 * 1000); gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C); gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C); gpio_pull_up(I2C_SDA_PIN); gpio_pull_up(I2C_SCL_PIN); bmp280_init(); 
    gpio_init(IN1_PIN); gpio_set_dir(IN1_PIN, GPIO_OUT); gpio_init(IN2_PIN); gpio_set_dir(IN2_PIN, GPIO_OUT); gpio_init(IN3_PIN); gpio_set_dir(IN3_PIN, GPIO_OUT); gpio_init(IN4_PIN); gpio_set_dir(IN4_PIN, GPIO_OUT); stop_motor(); 
    gpio_init(RAIN_SENSOR_DO_PIN); gpio_set_dir(RAIN_SENSOR_DO_PIN, GPIO_IN); gpio_init(PIR_OUT_PIN); gpio_set_dir(PIR_OUT_PIN, GPIO_IN); gpio_init(TCRT_DO_PIN); gpio_set_dir(TCRT_DO_PIN, GPIO_IN);

    if (cyw43_arch_init()) return 1;
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000)) return 1;
    printf("Wi-Fi OK.\n");

    mqtt_client = mqtt_client_new();
    mqtt_set_inpub_callback(mqtt_client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
    mqtt_do_connect();

    prev_motion = gpio_get(PIR_OUT_PIN);
    prev_obstacle = !gpio_get(TCRT_DO_PIN);
    prev_rain = !gpio_get(RAIN_SENSOR_DO_PIN);

    // Inicialização: Força abrir
    printf("INICIO: Abrindo janela...\n");
    move_window(true, "Inicializacao do Sistema");
    
    while(true) {
        #if PICO_CYW43_ARCH_POLL
        cyw43_arch_poll();
        #endif

        bool rain_curr = !gpio_get(RAIN_SENSOR_DO_PIN);
        bool motion_raw = gpio_get(PIR_OUT_PIN);
        bool obst_curr = !gpio_get(TCRT_DO_PIN);

        if (motion_raw) { 
            pir_filter_counter++; 
            if (pir_filter_counter >= CICLOS_FILTRO_MOVIMENTO) { motion_detected = true; pir_filter_counter = CICLOS_FILTRO_MOVIMENTO; } 
        } else { 
            pir_filter_counter = 0; motion_detected = false; 
        }
        
        is_raining = rain_curr;
        obstacle_detected = obst_curr;

        // -- ALERTAS DE MUDANÇA --
        if (obstacle_detected != prev_obstacle) {
            publicar(TOPICO_PUBLICA_OBSTACULO, obstacle_detected ? "DETECTADO" : "LIVRE");
            prev_obstacle = obstacle_detected;
        }
        if (motion_detected != prev_motion) {
            publicar(TOPICO_PUBLICA_MOVIMENTO, motion_detected ? "DETECTADO" : "LIVRE");
            prev_motion = motion_detected;
        }
        if (is_raining != prev_rain) {
            publicar(TOPICO_PUBLICA_CHUVA, is_raining ? "1" : "0");
            prev_rain = is_raining;
        }

        // -- PUBLICAÇÕES PERIÓDICAS E AVISOS DE SEGURANÇA --
        if (time_us_64() - last_temp_read_time > 2000000) { update_environment_data(); last_temp_read_time = time_us_64(); }

        if (time_us_64() - last_pub_time > 3000000) {
            if (mqtt_connected) {
                char temp_str[10]; sprintf(temp_str, "%.1f", current_temp_c);
                publicar(TOPICO_PUBLICA_TEMP, temp_str);
                publicar(TOPICO_PUBLICA_CHUVA, is_raining ? "1" : "0");
                
                // Dashboard Resumido
                printf("\n--- STATUS (3s) ---\n");
                printf("Temp: %s C | Chuva: %s | Modo: %s\n", temp_str, is_raining?"SIM":"NAO", override_ativo?"MANUAL":"AUTO");

                // --- AVISO DE SEGURANÇA NO DASHBOARD ---
                // Se estiver a chover e a janela estiver aberta, MAS bloqueada:
                if(window_is_open && is_raining && !override_ativo) {
                    if (motion_detected || obstacle_detected) {
                        printf(">> ALERTA: TENTANDO FECHAR MAS BLOQUEADO! <<\n");
                        publicar(TOPICO_MOTIVO_DETALHE, "ALERTA: Chuva mas Bloqueado!");
                    }
                }
            }
            last_pub_time = time_us_64();
        }

        // -- TIMER DO MODO MANUAL --
        if (override_ativo && to_us_since_boot(get_absolute_time()) > fim_do_override) {
            override_ativo = false;
            publicar(TOPICO_PUBLICA_MODO, "AUTO");
            publicar(TOPICO_MOTIVO_DETALHE, "Retorno ao Modo Automatico");
        }

        // -- LÓGICA DE CONTROLE --
        if (!override_ativo) {
            if (is_raining) {
                if (window_is_open && !motion_detected && !obstacle_detected) {
                    move_window(false, "Chuva Detectada!");
                }
            } else {
                if (current_temp_c > TEMP_PARA_ABRIR && !window_is_open) {
                    move_window(true, "Calor excessivo");
                } else if (current_temp_c < TEMP_PARA_FECHAR && window_is_open && !motion_detected && !obstacle_detected) {
                    move_window(false, "Temperatura baixa");
                }
            }
        }
        sleep_ms(10);
    }
    return 0;
}