#ifndef CONFIGURA_GERAL_H
#define CONFIGURA_GERAL_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"
#include <stdint.h>
#include <stdbool.h>

// ==========================================
// CONFIGURAÇÕES DE REDE
// ==========================================
#define WIFI_SSID "LFRenan"
#define WIFI_PASS "14192125lrf"

#define MQTT_BROKER_IP "192.168.15.46" 
#define MQTT_BROKER_PORT 1883

// ============================================================
// CONFIGURAÇÃO DO USUÁRIO
// ============================================================
#define TEMPO_OVERRIDE_MS 60000 

#define TEMP_PARA_ABRIR 33.0  
#define TEMP_PARA_FECHAR 25.0 

// ==========================================
// TÓPICOS MQTT (Adicionei Movimento e Obstáculo)
// ==========================================
#define TOPICO_PUBLICA_TEMP      "tcc/janela/temperatura"
#define TOPICO_PUBLICA_CHUVA     "tcc/janela/chuva"
#define TOPICO_PUBLICA_MODO      "tcc/janela/modo"
#define TOPICO_PUBLICA_STATUS    "tcc/janela/estado"
#define TOPICO_PUBLICA_MOVIMENTO "tcc/janela/movimento" 
#define TOPICO_PUBLICA_OBSTACULO "tcc/janela/obstaculo" 
#define TOPICO_SUBSCREVE_CMD     "tcc/janela/comando"
#define TOPICO_STATUS_SIMPLES "janela/status"   
#define TOPICO_MOTIVO_DETALHE "janela/detalhe"  

// ==========================================
// HARDWARE
// ==========================================
#define IN1_PIN 19
#define IN2_PIN 20
#define IN3_PIN 4
#define IN4_PIN 9
#define TOTAL_PASSOS 1366 

#define RAIN_SENSOR_DO_PIN 16
#define PIR_OUT_PIN 17
#define TCRT_DO_PIN 18 
#define CICLOS_FILTRO_MOVIMENTO 20 

#define I2C_PORT i2c1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
#define BMP280_ADDR 0x76

#endif