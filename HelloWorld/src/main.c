/*
 * - Teste de taxa de amostragem de 3000 Hz (intervalo ~333 µs)
 * - Quantificação de perda de amostras
 * - Escrita em cartão SD no formato CSV (time_ms, n_adc, adc_value)
 *
 * Plataforma: ESP32 (ESP-IDF via PlatformIO, board = esp32doit-devkit-v1)
 * 
 *
 */

// ==================== BIBLIOTECAS PADRÃO ====================
#include <stdio.h>      // Funções padrão de entrada/saída (printf, fprintf, etc.)
#include <stdlib.h>     // Funções utilitárias (atoi, malloc, free)
#include <string.h>     // Manipulação de strings (strlen, strftime)
#include <stdarg.h>     // Suporte a argumentos variáveis (vsnprintf)
#include <time.h>       // Funções de tempo (time, localtime, strftime)

// ==================== BIBLIOTECAS DO FREERTOS ====================
#include "freertos/FreeRTOS.h"   // Kernel do FreeRTOS
#include "freertos/task.h"       // Tarefas e delays (vTaskDelay)

// ==================== BIBLIOTECAS DO ESP-IDF ====================
#include "driver/uart.h"         // Driver para comunicação serial (UART)
#include "driver/gpio.h"         // Controle dos pinos GPIO
#include "esp_adc/adc_oneshot.h" // Driver para ADC no modo oneshot
#include "esp_adc/adc_cali.h"    // Calibração do ADC
#include "esp_adc/adc_cali_scheme.h" // Esquemas de calibração
#include "esp_timer.h"           // Timer de alta resolução do ESP32
#include "driver/sdmmc_host.h"   // Host para cartão SD via SDMMC
#include "driver/sdspi_host.h"   // Host para cartão SD via SPI
#include "sdmmc_cmd.h"           // Comandos para SDMMC
#include "esp_vfs_fat.h"         // Sistema de arquivos FAT no ESP32

// ================= CONFIGURAÇÃO DA UART =================
#define UART_PORT       UART_NUM_0      // Usa a UART0 (pinos padrão TX=GPIO1, RX=GPIO3)
#define TXD_PIN         GPIO_NUM_1      // Pino de transmissão (TX)
#define RXD_PIN         GPIO_NUM_3      // Pino de recepção (RX)
#define BUF_SIZE        1024            // Tamanho do buffer da UART
#define UART_TIMEOUT_MS 10000           // Timeout para leitura (10 segundos)

// ================= CONFIGURAÇÃO DO ADC =================
#define ADC1_CHAN0      ADC_CHANNEL_0   // Canal 0 do ADC1 (GPIO36)
#define ADC1_CHAN1      ADC_CHANNEL_3   // Canal 3 do ADC1 (GPIO39)
#define ADC_ATTEN       ADC_ATTEN_DB_12 
#define ADC_BITWIDTH    ADC_BITWIDTH_12 

// ================= CONFIGURAÇÃO DO TESTE DE ALTA TAXA =================
#define TAXA_AMOSTRAGEM_HZ      3000        // Taxa desejada: 3000 amostras por segundo (por canal)
#define INTERVALO_US            333         // Intervalo entre amostras: 1.000.000 µs / 3000 ≈ 333 µs
#define DURACAO_TESTE_SEGUNDOS  2           // Duração da coleta: 2 segundos
#define NUM_AMOSTRAS_TOTAL      30000       // Número total de amostras: 3000 Hz * 2 s * 2 canais = 12000
#define NUM_CANAIS              2           // Dois canais de ADC

// ================= CONFIGURAÇÃO DO SD CARD =================
#define PIN_NUM_MISO    19      // Pino MISO (Master In Slave Out) do barramento SPI
#define PIN_NUM_MOSI    23      // Pino MOSI (Master Out Slave In)
#define PIN_NUM_CLK     18      // Pino de clock (SCK)
#define PIN_NUM_CS      5       // Pino de chip select (CS) para o cartão SD
#define MOUNT_POINT     "/sdcard"  // Ponto de montagem no sistema de arquivos virtual

// ================= ESTRUTURA PARA ARMAZENAR AMOSTRAS =================
typedef struct {
    uint32_t timestamp_ms;      // Timestamp em milissegundos (tempo absoluto)
    uint8_t canal;              // Número do canal (0 ou 1)
    uint16_t valor;             // Valor do ADC (12 bits, 0-4095)
} amostra_t;

// ================= VARIÁVEIS GLOBAIS (compartilhadas com o callback do timer) =================
static amostra_t *amostras_buffer = NULL;          // Ponteiro para buffer dinâmico de amostras
static volatile int amostras_coletadas = 0;        // Contador de amostras já armazenadas (volátil para acesso em ISR)
static volatile bool coleta_finalizada = false;    // Flag indicando que a coleta terminou (buffer cheio ou tempo esgotado)
static const int MAX_AMOSTRAS = NUM_AMOSTRAS_TOTAL; // Tamanho máximo do buffer (constante)
static adc_oneshot_unit_handle_t adc_handle_global = NULL; // Handle do ADC (global para uso no callback)

// ================= FUNÇÕES DE COMUNICAÇÃO SERIAL =================
void uart_send_string(const char *str) {
    // Envia uma string pela UART usando a função do driver.
    uart_write_bytes(UART_PORT, str, strlen(str));
}

void uart_printf(const char *format, ...) {
    // Função similar ao printf, mas envia pela UART.
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args); // Formata a string
    va_end(args);
    uart_send_string(buffer); // Envia o buffer formatado
}

int uart_gets(char *buffer, int max_len) {
    // Lê uma linha do terminal (até ENTER) e armazena em 'buffer'.
    int idx = 0;
    while (idx < max_len - 1) {
        uint8_t c;
        int len = uart_read_bytes(UART_PORT, &c, 1, pdMS_TO_TICKS(UART_TIMEOUT_MS));
        if (len <= 0) {
            buffer[idx] = '\0';
            return -1; // Timeout
        }
        // Ignora caracteres de nova linha no início
        if ((c == '\n' || c == '\r') && idx == 0)
            continue;
        if (c == '\n' || c == '\r') {
            buffer[idx] = '\0';
            uart_printf("\n"); // Ecoa a quebra de linha
            return idx;
        }
        uart_write_bytes(UART_PORT, (const char *)&c, 1); // Ecoa o caractere digitado
        buffer[idx++] = (char)c;
    }
    buffer[idx] = '\0';
    return idx;
}

// ================= FUNÇÕES DE LOG =================
void log_erro(const char *mensagem) {
    // Exibe uma mensagem de erro no formato [ERRO] texto
    uart_printf("[ERRO] %s\n", mensagem);
}

void log_info(const char *mensagem) {
    // Exibe uma mensagem informativa no formato [INFO] texto
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
sdmmc_card_t* inicializar_sd_card(void) {
    // Inicializa o cartão SD no modo SPI, monta o sistema de arquivos e retorna o ponteiro do cartão.
    esp_err_t ret;
    sdmmc_card_t *card = NULL;

    // Configura o host SPI padrão para SD
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST; // Usa o periférico SPI2

    // Configuração do barramento SPI
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,   // Pino MOSI
        .miso_io_num = PIN_NUM_MISO,   // Pino MISO
        .sclk_io_num = PIN_NUM_CLK,    // Pino de clock
        .quadwp_io_num = -1,           // Não usado
        .quadhd_io_num = -1,           // Não usado
        .max_transfer_sz = 4000,       // Tamanho máximo de transferência
    };

    // Inicializa o barramento SPI com DMA padrão
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        log_erro("Falha ao inicializar barramento SPI para SD");
        return NULL;
    }

    // Configuração do dispositivo SD no modo SPI
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;   // Pino de chip select
    slot_config.host_id = host.slot;    // Mesmo host SPI

    // Configuração de montagem do sistema de arquivos FAT
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,  // Não formata automaticamente
        .max_files = 5,                   // Máximo de arquivos abertos simultaneamente
        .allocation_unit_size = 16 * 1024 // Tamanho do cluster (16KB)
    };

    // Monta o sistema de arquivos FAT no ponto especificado
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            log_erro("Falha ao montar sistema de arquivos. Verifique se o cartão está formatado em FAT.");
        } else {
            log_erro("Falha ao inicializar cartão SD (não conectado ou pinos incorretos).");
        }
        spi_bus_free(host.slot); // Libera o barramento SPI
        return NULL;
    }

    log_info("Cartão SD inicializado com sucesso!");
    uart_printf("Tamanho: %llu MB\n", ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024 * 1024));
    return card;
}

esp_err_t escrever_csv(const char *caminho_arquivo) {
    // Escreve o buffer de amostras em um arquivo CSV no cartão SD.
    FILE *f = fopen(caminho_arquivo, "w"); // Abre o arquivo para escrita (modo texto)
    if (f == NULL) {
        log_erro("Falha ao criar arquivo CSV");
        return ESP_FAIL;
    }

    // Escreve o cabeçalho do CSV
    fprintf(f, "time_ms,canal,valor_adc\n");

    // Percorre todas as amostras coletadas e escreve cada uma no arquivo
    for (int i = 0; i < amostras_coletadas; i++) {
        // Usa %lu para uint32_t, %u para uint8_t e uint16_t
        fprintf(f, "%lu,%u,%u\n",
                (unsigned long)amostras_buffer[i].timestamp_ms,
                amostras_buffer[i].canal,
                amostras_buffer[i].valor);
    }

    fclose(f); // Fecha o arquivo
    log_info("Arquivo CSV salvo com sucesso!");
    return ESP_OK;
}

// ================= CALLBACK DO TIMER =================
// Esta função será chamada pelo timer de hardware a cada INTERVALO_US (333 µs)
void timer_callback(void *arg) {
    // Verifica se a coleta já foi finalizada (buffer cheio ou tempo esgotado)
    if (coleta_finalizada) return;

    // Obtém o timestamp atual em microssegundos e converte para milissegundos
    uint64_t agora_us = esp_timer_get_time();
    uint32_t timestamp_ms = (uint32_t)(agora_us / 1000);

    int adc0_raw, adc1_raw;

    // Lê o canal 0 (ADC1_CHAN0)
    if (adc_oneshot_read(adc_handle_global, ADC1_CHAN0, &adc0_raw) == ESP_OK) {
        // Se a leitura foi bem-sucedida e ainda há espaço no buffer, armazena a amostra
        if (amostras_coletadas < MAX_AMOSTRAS) {
            amostras_buffer[amostras_coletadas].timestamp_ms = timestamp_ms;
            amostras_buffer[amostras_coletadas].canal = 0;
            amostras_buffer[amostras_coletadas].valor = (uint16_t)adc0_raw;
            amostras_coletadas++; // Incrementa o contador
        }
    }

    // Lê o canal 1 (ADC1_CHAN1)
    if (adc_oneshot_read(adc_handle_global, ADC1_CHAN1, &adc1_raw) == ESP_OK) {
        if (amostras_coletadas < MAX_AMOSTRAS) {
            amostras_buffer[amostras_coletadas].timestamp_ms = timestamp_ms;
            amostras_buffer[amostras_coletadas].canal = 1;
            amostras_buffer[amostras_coletadas].valor = (uint16_t)adc1_raw;
            amostras_coletadas++;
        }
    }

    // Se o buffer estiver cheio, sinaliza que a coleta terminou
    if (amostras_coletadas >= MAX_AMOSTRAS) {
        coleta_finalizada = true;
    }
}

// ================= TESTE DE ALTA TAXA DE AMOSTRAGEM =================
void testar_alta_taxa(adc_oneshot_unit_handle_t adc_handle) {
    // Exibe mensagens de início do teste
    uart_printf("\n===== INICIANDO TESTE DE ALTA TAXA (3000 Hz) =====\n");
    uart_printf("Intervalo entre amostras: %d µs\n", INTERVALO_US);
    uart_printf("Duração do teste: %d segundos\n", DURACAO_TESTE_SEGUNDOS);
    uart_printf("Amostras esperadas (2 canais): %d\n\n", NUM_AMOSTRAS_TOTAL);

    // Aloca dinamicamente o buffer para armazenar as amostras
    amostras_buffer = (amostra_t*)malloc(MAX_AMOSTRAS * sizeof(amostra_t));
    if (amostras_buffer == NULL) {
        log_erro("Falha ao alocar buffer de amostras");
        return;
    }

    // Inicializa as variáveis de controle
    amostras_coletadas = 0;
    coleta_finalizada = false;
    adc_handle_global = adc_handle;  // Guarda o handle do ADC para uso no callback

    // Configura o timer de hardware do ESP32
    esp_timer_handle_t timer;
    esp_timer_create_args_t timer_args = {
        .callback = timer_callback,   // Função callback que será chamada
        .arg = NULL,                  // Argumento (não usado)
        .name = "amostragem_timer"    // Nome para depuração
    };
    esp_err_t ret = esp_timer_create(&timer_args, &timer);
    if (ret != ESP_OK) {
        log_erro("Falha ao criar timer");
        free(amostras_buffer);
        amostras_buffer = NULL;
        return;
    }

    uint64_t tempo_inicio = esp_timer_get_time(); // Marca o início da coleta

    // Inicia o timer periódico com intervalo de INTERVALO_US microssegundos
    esp_timer_start_periodic(timer, INTERVALO_US);

    // Aguarda até que a coleta termine (buffer cheio ou tempo limite atingido)
    uint64_t tempo_limite = tempo_inicio + (DURACAO_TESTE_SEGUNDOS * 1000000ULL);
    while (!coleta_finalizada && esp_timer_get_time() < tempo_limite) {
        vTaskDelay(pdMS_TO_TICKS(1)); // Pequeno delay para não consumir toda a CPU
    }

    // Para o timer e libera seus recursos
    esp_timer_stop(timer);
    esp_timer_delete(timer);

    uint64_t tempo_fim = esp_timer_get_time();  // Marca o fim da coleta
    float tempo_real_segundos = (tempo_fim - tempo_inicio) / 1000000.0f; // Duração real em segundos

    // Calcula o número esperado de amostras com base no tempo real e na taxa teórica
    int amostras_esperadas = (int)(tempo_real_segundos * TAXA_AMOSTRAGEM_HZ * NUM_CANAIS);
    int amostras_reais = amostras_coletadas;
    float perda_percentual = 100.0f * (amostras_esperadas - amostras_reais) / amostras_esperadas;
    if (perda_percentual < 0) perda_percentual = 0; // Ajuste para evitar valores negativos

    // Exibe os resultados no terminal
    uart_printf("\n===== RESULTADOS DO TESTE =====\n");
    uart_printf("Tempo real de coleta: %.3f s\n", tempo_real_segundos);
    uart_printf("Amostras esperadas (2 canais): %d\n", amostras_esperadas);
    uart_printf("Amostras reais coletadas: %d\n", amostras_reais);
    uart_printf("Perda: %.2f%%\n", perda_percentual);
    uart_printf("Taxa efetiva: %.1f Hz\n", (amostras_reais / 2) / tempo_real_segundos);

    // Tenta salvar os dados no cartão SD
    log_info("Tentando salvar dados no SD card...");
    sdmmc_card_t *card = inicializar_sd_card();
    if (card != NULL) {
        // Gera um nome de arquivo único baseado na data/hora atual
        char caminho_arquivo[64];
        time_t agora = time(NULL);
        struct tm *tm_info = localtime(&agora);
        strftime(caminho_arquivo, sizeof(caminho_arquivo), MOUNT_POINT"/amostras_%Y%m%d_%H%M%S.csv", tm_info);

        // Escreve o arquivo CSV
        if (escrever_csv(caminho_arquivo) == ESP_OK) {
            uart_printf("Arquivo salvo: %s\n", caminho_arquivo);
        } else {
            log_erro("Falha ao escrever arquivo CSV");
        }

        // Desmonta o sistema de arquivos e libera os recursos do SD
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        spi_bus_free(SPI2_HOST);
    } else {
        log_erro("SD card não disponível. Dados não foram salvos.");
        // Caso não haja SD, exibe as primeiras 10 amostras no terminal para depuração
        uart_printf("\nPrimeiras 10 amostras (para depuração):\n");
        for (int i = 0; i < 10 && i < amostras_coletadas; i++) {
            uart_printf("[%lu] Canal %u: %u\n",
                       (unsigned long)amostras_buffer[i].timestamp_ms,
                       amostras_buffer[i].canal,
                       amostras_buffer[i].valor);
        }
    }

    // Libera a memória do buffer
    free(amostras_buffer);
    amostras_buffer = NULL;
    adc_handle_global = NULL;

    uart_printf("\n===== FIM DO TESTE =====\n");
}

// ================= PROGRAMA PRINCIPAL =================
void programa_principal(adc_oneshot_unit_handle_t adc_handle) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Pequena pausa inicial para estabilização

    uart_printf("\n===== INICIANDO PROGRAMA =====\n\n");
    uart_printf("Este programa demonstra:\n");
    uart_printf("1) Operações básicas (soma, par/ímpar)\n");
    uart_printf("2) Amostragem padrão a 2 Hz (como antes)\n");
    uart_printf("3) Teste de alta taxa (3000 Hz) com escrita em SD\n\n");

    char buffer[50];
    int numero1 = 0, numero2 = 0;

    // Solicita e lê o primeiro número
    uart_printf("Digite o primeiro numero: ");
    if (uart_gets(buffer, sizeof(buffer)) > 0) {
        numero1 = atoi(buffer);
    } else {
        log_erro("Falha ao ler o primeiro numero. Usando 0.");
        numero1 = 0;
    }

    // Solicita e lê o segundo número
    uart_printf("\nDigite o segundo numero: ");
    if (uart_gets(buffer, sizeof(buffer)) > 0) {
        numero2 = atoi(buffer);
    } else {
        log_erro("Falha ao ler o segundo numero. Usando 0.");
        numero2 = 0;
    }

    // Realiza a soma e verifica paridade
    int resultado = somar(numero1, numero2);
    uart_printf("\nA soma %d + %d = %d\n", numero1, numero2, resultado);
    verificarPar(resultado);

    // Amostragem padrão a 2 Hz (5 amostras, uma a cada 500 ms)
    uart_printf("\n--- Amostragem padrao (2 Hz) ---\n");
    for (int i = 0; i < 5; i++) {
        int adc0_raw, adc1_raw;
        adc_oneshot_read(adc_handle, ADC1_CHAN0, &adc0_raw);
        adc_oneshot_read(adc_handle, ADC1_CHAN1, &adc1_raw);
        uint64_t tempo_us = esp_timer_get_time();
        uint32_t tempo_ms = (uint32_t)(tempo_us / 1000);
        uart_printf("[%u ms] ADC0: %4d | ADC1: %4d\n", tempo_ms, adc0_raw, adc1_raw);
        vTaskDelay(pdMS_TO_TICKS(500)); // Atraso de 500 ms
    }

    // Executa o teste de alta taxa de amostragem
    testar_alta_taxa(adc_handle);

    uart_printf("\n===== FIM DO PROGRAMA =====\n");
}

// ================= FUNÇÃO PRINCIPAL =================
void app_main(void) {
    // -------------------- Configuração da UART --------------------
    uart_config_t uart_config = {
        .baud_rate = 115200,                 // Velocidade de comunicação
        .data_bits = UART_DATA_8_BITS,       // 8 bits de dados
        .parity = UART_PARITY_DISABLE,       // Sem paridade
        .stop_bits = UART_STOP_BITS_1,       // 1 bit de parada
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // Sem controle de fluxo
        .source_clk = UART_SCLK_APB,         // Clock da APB
    };

    // Aplica a configuração à UART
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0); // Instala o driver
    vTaskDelay(pdMS_TO_TICKS(1000)); // Aguarda estabilização
    log_info("UART inicializada.");

    // -------------------- Configuração do ADC --------------------
    adc_oneshot_unit_handle_t adc_handle;  // Handle da unidade ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,            // Usa ADC1
        .ulp_mode = ADC_ULP_MODE_DISABLE, // Modo ULP desabilitado
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        log_erro("Falha ao criar unidade ADC");
        while (1) vTaskDelay(1000); // Loop infinito em caso de erro
    }

    // Configuração do canal: atenuação e resolução
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ret = adc_oneshot_config_channel(adc_handle, ADC1_CHAN0, &chan_config);
    if (ret != ESP_OK) log_erro("Falha config canal0");
    ret = adc_oneshot_config_channel(adc_handle, ADC1_CHAN1, &chan_config);
    if (ret != ESP_OK) log_erro("Falha config canal1");

    log_info("ADC configurado.");

    // -------------------- Loop infinito do programa --------------------
    while (1) {
        programa_principal(adc_handle);  // Executa o programa principal

        // Aguarda 10 segundos antes de reiniciar o programa
        uart_printf("\nReiniciando programa em 10 segundos...\n");
        for (int i = 10; i >= 1; i--) {
            uart_printf("%d...\n", i);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        uart_flush(UART_PORT); // Limpa o buffer da UART
    }

    // Esta linha nunca será executada devido ao loop infinito, mas mantida para completude
    adc_oneshot_del_unit(adc_handle);
}