/* 
 * - Teste de taxa de amostragem de 3000 Hz (intervalo ~333 µs)
 * - Quantificação de perda de amostras
 * - Escrita em cartão SD no formato CSV (time_ms, n_adc, adc_value)
 * 
 * Plataforma: ESP32 (ESP-IDF via PlatformIO, board = esp32doit-devkit-v1)
 * Autor: João Nogueira
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

// ================= CONFIGURAÇÃO DA UART =================
#define UART_PORT       UART_NUM_0
#define TXD_PIN         GPIO_NUM_1
#define RXD_PIN         GPIO_NUM_3
#define BUF_SIZE        1024
#define UART_TIMEOUT_MS 10000

// ================= CONFIGURAÇÃO DO ADC =================
#define ADC1_CHAN0      ADC_CHANNEL_0   // GPIO36
#define ADC1_CHAN1      ADC_CHANNEL_3   // GPIO39
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define ADC_BITWIDTH    ADC_BITWIDTH_12

// ================= CONFIGURAÇÃO DO TESTE DE ALTA TAXA =================
#define TAXA_AMOSTRAGEM_HZ      3000                // 3000 Hz
#define INTERVALO_US            333                 // 1.000.000 / 3000 ≈ 333 µs
#define DURACAO_TESTE_SEGUNDOS  5                   // Tempo de coleta (5 segundos)
#define NUM_AMOSTRAS_TOTAL      30000               // 3000 Hz * 5 s * 2 canais = 30000
#define NUM_CANAIS              2

// ================= CONFIGURAÇÃO DO SD CARD =================
#define PIN_NUM_MISO    19
#define PIN_NUM_MOSI    23
#define PIN_NUM_CLK     18
#define PIN_NUM_CS      5
#define MOUNT_POINT     "/sdcard"

// ================= ESTRUTURA PARA ARMAZENAR AMOSTRAS =================
typedef struct {
    uint32_t timestamp_ms;      // Timestamp em milissegundos
    uint8_t canal;              // Número do canal (0 ou 1)
    uint16_t valor;             // Valor do ADC (12 bits)
} amostra_t;

// Buffer circular para armazenar amostras durante a coleta
static amostra_t *amostras_buffer = NULL;
static volatile int amostras_coletadas = 0;
static const int MAX_AMOSTRAS = NUM_AMOSTRAS_TOTAL;

// ================= FUNÇÕES DE COMUNICAÇÃO SERIAL =================
void uart_send_string(const char *str) {
    uart_write_bytes(UART_PORT, str, strlen(str));
}

void uart_printf(const char *format, ...) {
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    uart_send_string(buffer);
}

int uart_gets(char *buffer, int max_len) {
    int idx = 0;
    while (idx < max_len - 1) {
        uint8_t c;
        int len = uart_read_bytes(UART_PORT, &c, 1, pdMS_TO_TICKS(UART_TIMEOUT_MS));
        if (len <= 0) {
            buffer[idx] = '\0';
            return -1;
        }
        if ((c == '\n' || c == '\r') && idx == 0)
            continue;
        if (c == '\n' || c == '\r') {
            buffer[idx] = '\0';
            uart_printf("\n");
            return idx;
        }
        uart_write_bytes(UART_PORT, (const char *)&c, 1);
        buffer[idx++] = (char)c;
    }
    buffer[idx] = '\0';
    return idx;
}

// ================= FUNÇÕES DE LOG =================
void log_erro(const char *mensagem) {
    uart_printf("[ERRO] %s\n", mensagem);
}

void log_info(const char *mensagem) {
    uart_printf("[INFO] %s\n", mensagem);
}

// ================= FUNÇÕES DO PROGRAMA ORIGINAL =================
int somar(int a, int b) {
    return a + b;
}

void verificarPar(int numero) {
    if (numero % 2 == 0)
        uart_printf("O numero %d eh PAR.\n", numero);
    else
        uart_printf("O numero %d eh IMPAR.\n", numero);
}

// ================= FUNÇÕES PARA SD CARD =================
// Inicializa o cartão SD no modo SPI
sdmmc_card_t* inicializar_sd_card(void) {
    esp_err_t ret;
    sdmmc_card_t *card = NULL;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        log_erro("Falha ao inicializar barramento SPI para SD");
        return NULL;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            log_erro("Falha ao montar sistema de arquivos. Verifique se o cartão está formatado em FAT.");
        } else {
            log_erro("Falha ao inicializar cartão SD (não conectado ou pinos incorretos).");
        }
        spi_bus_free(host.slot);
        return NULL;
    }

    log_info("Cartão SD inicializado com sucesso!");
    uart_printf("Tamanho: %llu MB\n", ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024 * 1024));
    return card;
}

// Escreve o buffer de amostras no arquivo CSV
esp_err_t escrever_csv(const char *caminho_arquivo) {
    FILE *f = fopen(caminho_arquivo, "w");
    if (f == NULL) {
        log_erro("Falha ao criar arquivo CSV");
        return ESP_FAIL;
    }

    // Escreve cabeçalho
    fprintf(f, "time_ms,canal,valor_adc\n");

    // Escreve todas as amostras coletadas
    for (int i = 0; i < amostras_coletadas; i++) {
        // Usa %lu para uint32_t (timestamp) e %u para uint8_t e uint16_t
        fprintf(f, "%lu,%u,%u\n", 
                (unsigned long)amostras_buffer[i].timestamp_ms,
                amostras_buffer[i].canal,
                amostras_buffer[i].valor);
    }

    fclose(f);
    log_info("Arquivo CSV salvo com sucesso!");
    return ESP_OK;
}

// ================= TESTE DE ALTA TAXA DE AMOSTRAGEM =================
void testar_alta_taxa(adc_oneshot_unit_handle_t adc_handle) {
    uart_printf("\n===== INICIANDO TESTE DE ALTA TAXA (3000 Hz) =====\n");
    uart_printf("Intervalo entre amostras: %d µs\n", INTERVALO_US);
    uart_printf("Duração do teste: %d segundos\n", DURACAO_TESTE_SEGUNDOS);
    uart_printf("Amostras esperadas (2 canais): %d\n\n", NUM_AMOSTRAS_TOTAL);

    // Aloca buffer para armazenar as amostras
    amostras_buffer = (amostra_t*)malloc(MAX_AMOSTRAS * sizeof(amostra_t));
    if (amostras_buffer == NULL) {
        log_erro("Falha ao alocar buffer de amostras");
        return;
    }

    amostras_coletadas = 0;
    uint64_t tempo_inicio = esp_timer_get_time();
    uint64_t tempo_limite = tempo_inicio + (DURACAO_TESTE_SEGUNDOS * 1000000ULL);
    uint64_t proxima_amostra = tempo_inicio;
    int amostras_esperadas = 0;
    int adc0_raw, adc1_raw;   // Declaração das variáveis para leitura

    // Loop principal de coleta
    while (esp_timer_get_time() < tempo_limite && amostras_coletadas < MAX_AMOSTRAS - 1) {
        // Espera ativa até o próximo instante de amostragem
        while (esp_timer_get_time() < proxima_amostra) {
            // Pequeno pause para não sobrecarregar a CPU
            asm volatile ("nop");
        }

        // Calcula timestamp atual em ms
        uint64_t agora_us = esp_timer_get_time();
        uint32_t timestamp_ms = (uint32_t)(agora_us / 1000);

        // Lê canal 0
        esp_err_t ret = adc_oneshot_read(adc_handle, ADC1_CHAN0, &adc0_raw);
        if (ret == ESP_OK) {
            amostras_buffer[amostras_coletadas].timestamp_ms = timestamp_ms;
            amostras_buffer[amostras_coletadas].canal = 0;
            amostras_buffer[amostras_coletadas].valor = (uint16_t)adc0_raw;
            amostras_coletadas++;
        }

        // Lê canal 1
        ret = adc_oneshot_read(adc_handle, ADC1_CHAN1, &adc1_raw);
        if (ret == ESP_OK && amostras_coletadas < MAX_AMOSTRAS) {
            amostras_buffer[amostras_coletadas].timestamp_ms = timestamp_ms;
            amostras_buffer[amostras_coletadas].canal = 1;
            amostras_buffer[amostras_coletadas].valor = (uint16_t)adc1_raw;
            amostras_coletadas++;
        }

        amostras_esperadas += 2;  // Duas leituras por ciclo
        proxima_amostra += INTERVALO_US;
    }

    uint64_t tempo_fim = esp_timer_get_time();
    float tempo_real_segundos = (tempo_fim - tempo_inicio) / 1000000.0f;

    // Calcula perda
    int amostras_reais = amostras_coletadas;
    float perda_percentual = 100.0f * (amostras_esperadas - amostras_reais) / amostras_esperadas;
    if (perda_percentual < 0) perda_percentual = 0;

    uart_printf("\n===== RESULTADOS DO TESTE =====\n");
    uart_printf("Tempo real de coleta: %.3f s\n", tempo_real_segundos);
    uart_printf("Amostras esperadas (2 canais): %d\n", amostras_esperadas);
    uart_printf("Amostras reais coletadas: %d\n", amostras_reais);
    uart_printf("Perda: %.2f%%\n", perda_percentual);
    uart_printf("Taxa efetiva: %.1f Hz\n", (amostras_reais / 2) / tempo_real_segundos);

    // Salva no SD card se disponível
    log_info("Tentando salvar dados no SD card...");
    sdmmc_card_t *card = inicializar_sd_card();
    if (card != NULL) {
        char caminho_arquivo[64];
        // Gera nome único com timestamp
        time_t agora = time(NULL);
        struct tm *tm_info = localtime(&agora);
        strftime(caminho_arquivo, sizeof(caminho_arquivo), MOUNT_POINT"/amostras_%Y%m%d_%H%M%S.csv", tm_info);
        
        if (escrever_csv(caminho_arquivo) == ESP_OK) {
            uart_printf("Arquivo salvo: %s\n", caminho_arquivo);
        } else {
            log_erro("Falha ao escrever arquivo CSV");
        }

        // Desmonta e libera
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        spi_bus_free(SPI2_HOST);
    } else {
        log_erro("SD card não disponível. Dados não foram salvos.");
        // Apenas para depuração, exibe as primeiras 10 amostras
        uart_printf("\nPrimeiras 10 amostras (para depuração):\n");
        for (int i = 0; i < 10 && i < amostras_coletadas; i++) {
            uart_printf("[%lu] Canal %u: %u\n", 
                       (unsigned long)amostras_buffer[i].timestamp_ms,
                       amostras_buffer[i].canal,
                       amostras_buffer[i].valor);
        }
    }

    // Libera buffer
    free(amostras_buffer);
    amostras_buffer = NULL;

    uart_printf("\n===== FIM DO TESTE =====\n");
}

// ================= PROGRAMA PRINCIPAL =================
void programa_principal(adc_oneshot_unit_handle_t adc_handle) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    uart_printf("\n===== INICIANDO PROGRAMA =====\n\n");
    uart_printf("Este programa demonstra:\n");
    uart_printf("1) Operações básicas (soma, par/ímpar)\n");
    uart_printf("2) Amostragem padrão a 2 Hz (como antes)\n");
    uart_printf("3) Teste de alta taxa (3000 Hz) com escrita em SD\n\n");

    char buffer[50];
    int numero1 = 0, numero2 = 0;

    // Entrada dos números (igual ao original)
    uart_printf("Digite o primeiro numero: ");
    if (uart_gets(buffer, sizeof(buffer)) > 0) {
        numero1 = atoi(buffer);
    } else {
        log_erro("Falha ao ler o primeiro numero. Usando 0.");
        numero1 = 0;
    }

    uart_printf("\nDigite o segundo numero: ");
    if (uart_gets(buffer, sizeof(buffer)) > 0) {
        numero2 = atoi(buffer);
    } else {
        log_erro("Falha ao ler o segundo numero. Usando 0.");
        numero2 = 0;
    }

    int resultado = somar(numero1, numero2);
    uart_printf("\nA soma %d + %d = %d\n", numero1, numero2, resultado);
    verificarPar(resultado);

    // Amostragem padrão a 2 Hz (mantida do original)
    uart_printf("\n--- Amostragem padrao (2 Hz) ---\n");
    for (int i = 0; i < 5; i++) {
        int adc0_raw, adc1_raw;
        adc_oneshot_read(adc_handle, ADC1_CHAN0, &adc0_raw);
        adc_oneshot_read(adc_handle, ADC1_CHAN1, &adc1_raw);
        uint64_t tempo_us = esp_timer_get_time();
        uint32_t tempo_ms = (uint32_t)(tempo_us / 1000);
        uart_printf("[%u ms] ADC0: %4d | ADC1: %4d\n", tempo_ms, adc0_raw, adc1_raw);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Teste de alta taxa
    testar_alta_taxa(adc_handle);

    uart_printf("\n===== FIM DO PROGRAMA =====\n");
}

// ================= FUNÇÃO PRINCIPAL =================
void app_main(void) {
    // -------------------- UART --------------------
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    log_info("UART inicializada.");

    // -------------------- ADC --------------------
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        log_erro("Falha ao criar unidade ADC");
        while (1) vTaskDelay(1000);
    }

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ret = adc_oneshot_config_channel(adc_handle, ADC1_CHAN0, &chan_config);
    if (ret != ESP_OK) log_erro("Falha config canal0");
    ret = adc_oneshot_config_channel(adc_handle, ADC1_CHAN1, &chan_config);
    if (ret != ESP_OK) log_erro("Falha config canal1");

    log_info("ADC configurado.");

    // -------------------- Loop infinito --------------------
    while (1) {
        programa_principal(adc_handle);

        uart_printf("\nReiniciando programa em 10 segundos...\n");
        for (int i = 10; i >= 1; i--) {
            uart_printf("%d...\n", i);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        uart_flush(UART_PORT);
    }

    adc_oneshot_del_unit(adc_handle); // Nunca alcançado
}