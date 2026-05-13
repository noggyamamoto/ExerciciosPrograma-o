/*
 * ============================================================================
 * Projeto integrado para ESP32 (ESP-IDF via PlatformIO) - COLETA CONTÍNUA
 * Plataforma: esp32doit-devkit-v1
 *
 * Características:
 *  - 8 canais analógicos (GPIO32 a GPIO39) a 3000 Hz cada -> 24000 amostras/s
 *  - Aquisição com DMA (ADC contínuo) no Core 1
 *  - Blocos de 4096 amostras, buffer circular com 4 blocos (32 KB cada)
 *  - Tarefa escritora no Core 0 grava em binário com CRC-32
 *  - Coleta infinita: iniciada/parada pelo menu (opções 5 e 6)
 *  - Montagem do cartão SD em "/sdcard" (padrão), com nomes curtos compatíveis FAT 8.3
 * ============================================================================
 */

// =========================== BIBLIOTECAS PADRÃO ===========================
#include <stdio.h>      // Funções padrão de entrada/saída (fopen, fwrite, etc.)
#include <stdlib.h>     // Funções utilitárias (malloc, free, atoi)
#include <string.h>     // Manipulação de strings (memset, snprintf)
#include <stdarg.h>     // Para argumentos variáveis (vsnprintf no uart_printf)
#include <time.h>       // Funções de data/hora (não usado diretamente)
#include <stdbool.h>    // Tipo booleano (true/false)
#include <errno.h>      // Códigos de erro (errno, strerror)

// =========================== BIBLIOTECAS DO FREERTOS ======================
#include "freertos/FreeRTOS.h"     // Kernel do FreeRTOS (tasks, mutex, delays)
#include "freertos/task.h"         // Funções de tarefas (xTaskCreate, vTaskDelay)
#include "freertos/semphr.h"       // Semáforos e mutex (xSemaphoreCreateMutex)

// =========================== BIBLIOTECAS DO ESP-IDF =======================
#include "driver/uart.h"           // Driver da UART (comunicação serial)
#include "driver/gpio.h"           // Controle de GPIO (pinos)
#include "driver/spi_master.h"     // Driver SPI (para comunicação com o SD)
#include "esp_timer.h"             // Timer de alta resolução (microssegundos)
#include "esp_heap_caps.h"         // Funções de memória heap (esp_get_free_heap)
#include "esp_rom_crc.h"           // Cálculo de CRC-32 via ROM (mais rápido)

#include "esp_adc/adc_oneshot.h"   // ADC oneshot (para testes simples)
#include "esp_adc/adc_continuous.h"// ADC contínuo com DMA (para alta taxa)
#include "driver/sdspi_host.h"     // Driver SD via SPI
#include "sdmmc_cmd.h"             // Estruturas e comandos para cartão SD
#include "esp_vfs_fat.h"           // Sistema de arquivos FAT (montagem do SD)

// =========================== CONFIGURAÇÃO DA UART =========================
#define UART_PORT               UART_NUM_0     // UART0 para Serial Monitor
#define TXD_PIN                 GPIO_NUM_1     // Pino TX padrão do ESP32
#define RXD_PIN                 GPIO_NUM_3     // Pino RX padrão do ESP32
#define UART_RX_BUF_SIZE        2048           // Tamanho do buffer de recepção
#define UART_TX_BUF_SIZE        2048           // Tamanho do buffer de transmissão
#define UART_LINE_TIMEOUT_MS    10000          // Timeout máximo para ler uma linha (10s)

// =========================== CONFIGURAÇÃO DO ADC ONESHOT ==================
#define ADC1_CHAN0              ADC_CHANNEL_0  // Canal 0 do ADC1 no GPIO36
#define ADC1_CHAN1              ADC_CHANNEL_3  // Canal 3 do ADC1 no GPIO39
#define ADC_ATTEN               ADC_ATTEN_DB_12 // Atenuação de 12dB (0-3.3V)
#define ADC_ONESHOT_BITWIDTH    ADC_BITWIDTH_12 // 12 bits de resolução

// =========================== CONFIGURAÇÃO DO ADC CONTÍNUO (DMA) ============
#define ADC_CONTINUOUS_UNIT     ADC_UNIT_1      // ADC1
#define ADC_ATTEN_DB            ADC_ATTEN_DB_12 // Mesma atenuação
#define ADC_CONTINUOUS_BITWIDTH SOC_ADC_DIGI_MAX_BITWIDTH // 12 bits (valor máximo)

// Mapeamento dos 8 canais do ADC1 (GPIO32 a GPIO39)
#define ADC1_CH0                ADC_CHANNEL_0  // GPIO36
#define ADC1_CH1                ADC_CHANNEL_1  // GPIO37
#define ADC1_CH2                ADC_CHANNEL_2  // GPIO38
#define ADC1_CH3                ADC_CHANNEL_3  // GPIO39
#define ADC1_CH4                ADC_CHANNEL_4  // GPIO32
#define ADC1_CH5                ADC_CHANNEL_5  // GPIO33
#define ADC1_CH6                ADC_CHANNEL_6  // GPIO34
#define ADC1_CH7                ADC_CHANNEL_7  // GPIO35

// =========================== PARÂMETROS DA AQUISIÇÃO ======================
#define NUM_CHANNELS            8                     // Número de canais
#define SAMPLE_RATE_PER_CHANNEL 3000                  // 3000 Hz por canal
#define HIGH_RATE_TOTAL_HZ      (NUM_CHANNELS * SAMPLE_RATE_PER_CHANNEL) // 24000 Hz total

// =========================== DOUBLE BUFFER CIRCULAR ========================
#define BLOCK_SAMPLES           4096                  // Amostras por bloco (4096 * 8 = 32KB)
#define BLOCK_BYTES             (BLOCK_SAMPLES * sizeof(amostra_t)) // 32768 bytes
#define CIRC_BUFFER_CAPACITY    4                     // Nº máximo de blocos em RAM
#define DMA_BUFFER_SIZE         8192                  // Buffer interno do driver DMA

// =========================== CONFIGURAÇÃO DO SD CARD ======================
#define MOUNT_POINT             "/sdcard"          // Ponto de montagem padrão
#define PIN_NUM_MISO            19                // MISO
#define PIN_NUM_MOSI            23                // MOSI
#define PIN_NUM_CLK             18                // SCK
#define PIN_NUM_CS              5                 // CS (chip select)
#define SPI_SPEED_HZ            4000000            // 4 MHz

// =========================== ESTRUTURAS DE DADOS ==========================
// Estrutura de uma amostra individual (com alinhamento natural de 8 bytes)
typedef struct {
    uint32_t time_ms;    // Timestamp em milissegundos (teórico)
    uint8_t  n_adc;      // Número do canal (0..7)
    uint16_t adc_value;  // Valor bruto do ADC (0..4095)
} amostra_t;

// Estrutura que representa um bloco de amostras (alocado dinamicamente)
typedef struct {
    amostra_t* data;        // Ponteiro para o array de amostras
    uint32_t   num_samples; // Número de amostras válidas neste bloco
} block_t;

// Cabeçalho do arquivo binário (atributo packed elimina padding entre campos)
typedef struct __attribute__((packed)) {
    uint32_t magic;          // Número mágico para identificar o formato (0xCEFAEDFE)
    uint32_t version;        // Versão do formato (1)
    uint32_t block_num;      // Número sequencial do bloco
    uint64_t start_time_us;  // Timestamp de início da coleta (microssegundos)
    uint32_t num_samples;    // Quantidade de amostras neste bloco
    uint32_t crc;            // CRC-32 do restante do arquivo (exceto este campo)
} block_header_t;

// =========================== VARIÁVEIS GLOBAIS =============================
// Buffer circular (FIFO de ponteiros para blocos)
static block_t*   g_circ_buffer[CIRC_BUFFER_CAPACITY]; // Vetor de ponteiros
static int        g_buffer_head = 0;      // Índice de leitura (consumidor)
static int        g_buffer_tail = 0;      // Índice de escrita (produtor)
static int        g_buffer_count = 0;     // Número de elementos ocupados
static SemaphoreHandle_t g_buffer_mutex = NULL;  // Mutex para acesso concorrente

// Handles dos periféricos
static adc_oneshot_unit_handle_t   g_oneshot_handle = NULL;  // ADC oneshot
static adc_continuous_handle_t     g_cont_handle = NULL;     // ADC contínuo
static TaskHandle_t                g_acquisition_task_handle = NULL; // Tarefa produtora
static TaskHandle_t                g_writer_task_handle = NULL;      // Tarefa consumidora

// Flags de controle da coleta
static volatile bool g_acquisition_running = false;  // Indica se a coleta está ativa
static volatile bool g_stop_request = false;         // Sinal para parar as tarefas
static uint64_t      g_start_time_us = 0;            // Tempo de início (us)
static uint32_t      g_block_sequence = 0;           // Próximo número de bloco a ser usado

// Estatísticas para depuração
static volatile uint32_t g_callback_count = 0;   // Número de vezes que o callback ISR foi chamado
static volatile uint32_t g_total_samples = 0;    // Total de amostras processadas

// =========================== PROTÓTIPOS ====================================
static void uart_send_string(const char *str);
static void uart_printf(const char *format, ...);
static int uart_gets(char *buffer, int max_len);
static void log_erro(const char *mensagem);
static void log_info(const char *mensagem);
static void mostrar_memoria_livre(const char *mensagem);
static int somar(int a, int b);
static void verificarPar(int numero);
static sdmmc_card_t *inicializar_sd_card(void);
static esp_err_t escrever_bloco_binario(const block_t* bloco, uint32_t seq);
static void teste_entrada_saida_serial(void);
static void teste_leitura_adc_simples(void);
static void teste_leitura_adc_duplo(void);
static void teste_amostragem_periodica_1_canal(uint32_t frequencia_hz, uint8_t canal);
static void start_continuous_acquisition(void);
static void stop_continuous_acquisition(void);
static void adc_acquisition_task(void *pvParameters);
static void sd_writer_task(void *pvParameters);
static bool continuous_adc_callback(adc_continuous_handle_t handle,
                                    const adc_continuous_evt_data_t *edata,
                                    void *user_data);  
static void imprimir_menu(void);
static void comunicacao_task(void *pvParameters);
static void continuous_adc_init(adc_continuous_handle_t *out_handle);
static bool circ_buffer_push(block_t* bloco);
static block_t* circ_buffer_pop(void);

// =========================== FUNÇÕES UART ==================================
// Envia uma string pela UART (escrita direta)
static void uart_send_string(const char *str) {
    uart_write_bytes(UART_PORT, str, strlen(str));
}

// printf-like via UART (usa buffer interno de 512 bytes)
static void uart_printf(const char *format, ...) {
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    uart_send_string(buffer);
}

// Lê uma linha da UART até ENTER, com timeout.
// Retorna número de caracteres lidos ou -1 em caso de timeout.
static int uart_gets(char *buffer, int max_len) {
    int idx = 0;
    while (idx < max_len - 1) {
        uint8_t c;
        int len = uart_read_bytes(UART_PORT, &c, 1, pdMS_TO_TICKS(UART_LINE_TIMEOUT_MS));
        if (len <= 0) {                // Timeout sem receber dados
            buffer[idx] = '\0';
            return -1;
        }
        // Ignora ENTER no início da linha
        if ((c == '\n' || c == '\r') && idx == 0) continue;
        // Finaliza a linha ao encontrar ENTER
        if (c == '\n' || c == '\r') {
            buffer[idx] = '\0';
            uart_printf("\n");
            return idx;
        }
        uart_write_bytes(UART_PORT, (const char *)&c, 1);  // Ecoa o caractere
        buffer[idx++] = (char)c;
    }
    buffer[idx] = '\0';
    return idx;
}

// =========================== LOGS =========================================
static void log_erro(const char *mensagem) {
    uart_printf("[ERRO] %s\n", mensagem);
}
static void log_info(const char *mensagem) {
    uart_printf("[INFO] %s\n", mensagem);
}
static void mostrar_memoria_livre(const char *mensagem) {
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    uart_printf("[MEM] %s: %u bytes livres\n", mensagem, (unsigned int)free_heap);
}

// =========================== FUNÇÕES BÁSICAS ==============================
static int somar(int a, int b) { return a + b; }
static void verificarPar(int numero) {
    if (numero % 2 == 0)
        uart_printf("O numero %d e PAR.\n", numero);
    else
        uart_printf("O numero %d e IMPAR.\n", numero);
}

// =========================== SD CARD (MONTAGEM) ===========================
// Inicializa o cartão SD, monta sistema de arquivos FAT e retorna ponteiro.
static sdmmc_card_t *inicializar_sd_card(void) {
    esp_err_t ret;
    sdmmc_card_t *card = NULL;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;                        // Usa SPI3
    host.max_freq_khz = SPI_SPEED_HZ / 1000;     // 4 MHz

    // Configuração do barramento SPI
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        log_erro("Falha ao inicializar barramento SPI para SD");
        return NULL;
    }

    // Configuração do pino CS para o SD
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    // Configuração do sistema de arquivos FAT
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    // Monta o cartão SD no diretório "/sdcard"
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL)
            log_erro("Falha ao montar o sistema de arquivos FAT.");
        else
            log_erro("Falha ao inicializar o cartao SD.");
        spi_bus_free(host.slot);
        return NULL;
    }

    log_info("Cartao SD inicializado com sucesso (montado em /sdcard)!");
    uart_printf("Capacidade aproximada: %llu MB\n",
                ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024ULL * 1024ULL));

    // Teste de escrita no diretório
    FILE *test = fopen(MOUNT_POINT "/test.txt", "w");
    if (test) {
        fputs("teste", test);
        fclose(test);
        remove(MOUNT_POINT "/test.txt");
        log_info("SD gravável em /sdcard");
    } else {
        log_erro("Não foi possível escrever em /sdcard");
    }
    return card;
}

// =========================== ESCRITA DE BLOCO (NOME CURTO) ================
// Grava um bloco de amostras em arquivo binário com cabeçalho e CRC.
static esp_err_t escrever_bloco_binario(const block_t* bloco, uint32_t seq) {
    if (bloco == NULL || bloco->data == NULL) return ESP_FAIL;

    char filename[64];
    // Nome curto no formato 8.3: /sdcard/b0000.bin (compatível com FAT)
    snprintf(filename, sizeof(filename), MOUNT_POINT "/b%04u.bin", (unsigned int)seq);

    FILE *f = fopen(filename, "wb");
    if (f == NULL) {
        uart_printf("[ERRO] fopen falhou: %s (errno=%d)\n", strerror(errno), errno);
        return ESP_FAIL;
    }

    block_header_t header;
    header.magic        = 0xCEFAEDFE;
    header.version      = 1;
    header.block_num    = seq;
    header.start_time_us = g_start_time_us;
    header.num_samples  = bloco->num_samples;
    header.crc          = 0;   // temporário

    // Cálculo do CRC sobre o cabeçalho (exceto campo crc) + dados
    uint32_t crc = esp_rom_crc32_le(~0U, (uint8_t*)&header, offsetof(block_header_t, crc));
    crc = esp_rom_crc32_le(crc, (uint8_t*)bloco->data, bloco->num_samples * sizeof(amostra_t));
    header.crc = ~crc;                     // complemento final

    fwrite(&header, sizeof(header), 1, f);
    fwrite(bloco->data, sizeof(amostra_t), bloco->num_samples, f);
    fclose(f);
    return ESP_OK;
}

// =========================== BUFFER CIRCULAR ==============================
// Insere um bloco no buffer circular. Retorna true se sucesso, false se cheio.
static bool circ_buffer_push(block_t* bloco) {
    if (g_buffer_count >= CIRC_BUFFER_CAPACITY) return false;
    g_circ_buffer[g_buffer_tail] = bloco;
    g_buffer_tail = (g_buffer_tail + 1) % CIRC_BUFFER_CAPACITY;
    g_buffer_count++;
    return true;
}

// Remove e retorna um bloco do buffer. Retorna NULL se vazio.
static block_t* circ_buffer_pop(void) {
    if (g_buffer_count == 0) return NULL;
    block_t* bloco = g_circ_buffer[g_buffer_head];
    g_buffer_head = (g_buffer_head + 1) % CIRC_BUFFER_CAPACITY;
    g_buffer_count--;
    return bloco;
}

// =========================== TESTES BÁSICOS ===============================
static void teste_entrada_saida_serial(void) {
    char buffer[64];
    uart_printf("\n===== TESTE DE ENTRADA E SAIDA =====\n");
    uart_printf("Digite o primeiro numero: ");
    int numero1 = 0;
    if (uart_gets(buffer, sizeof(buffer)) > 0) numero1 = atoi(buffer);
    else log_erro("Timeout");
    uart_printf("Digite o segundo numero: ");
    int numero2 = 0;
    if (uart_gets(buffer, sizeof(buffer)) > 0) numero2 = atoi(buffer);
    else log_erro("Timeout");
    int resultado = somar(numero1, numero2);
    uart_printf("\nResultado: %d + %d = %d\n", numero1, numero2, resultado);
    verificarPar(resultado);
}

static void teste_leitura_adc_simples(void) {
    uart_printf("\n===== LEITURA DE 1 PORTA ANALOGICA =====\n");
    int adc_raw = 0;
    if (adc_oneshot_read(g_oneshot_handle, ADC1_CHAN0, &adc_raw) == ESP_OK)
        uart_printf("ADC0 (GPIO36) = %d\n", adc_raw);
    else log_erro("Falha");
}

static void teste_leitura_adc_duplo(void) {
    uart_printf("\n===== LEITURA DE 2 PORTAS ANALOGICAS =====\n");
    int adc0_raw = 0, adc1_raw = 0;
    if (adc_oneshot_read(g_oneshot_handle, ADC1_CHAN0, &adc0_raw) != ESP_OK)
        log_erro("Falha ADC0");
    if (adc_oneshot_read(g_oneshot_handle, ADC1_CHAN1, &adc1_raw) != ESP_OK)
        log_erro("Falha ADC1");
    uart_printf("ADC0 (GPIO36) = %d\n", adc0_raw);
    uart_printf("ADC1 (GPIO39) = %d\n", adc1_raw);
}

static void teste_amostragem_periodica_1_canal(uint32_t frequencia_hz, uint8_t canal) {
    uart_printf("\n===== AMOSTRAGEM PERIODICA DE 1 CANAL =====\n");
    uart_printf("Freq: %u Hz, Canal: %u\n", (unsigned int)frequencia_hz, (unsigned int)canal);
    uint64_t periodo_us = 1000000ULL / frequencia_hz;
    uint64_t proximo = esp_timer_get_time();
    for (int i = 0; i < 5; i++) {
        proximo += periodo_us;
        while (esp_timer_get_time() < proximo) vTaskDelay(pdMS_TO_TICKS(1));
        int adc_raw = 0;
        esp_err_t ret = (canal == 0) ? adc_oneshot_read(g_oneshot_handle, ADC1_CHAN0, &adc_raw)
                                     : adc_oneshot_read(g_oneshot_handle, ADC1_CHAN1, &adc_raw);
        uint32_t tempo_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        if (ret == ESP_OK)
            uart_printf("[%u ms] Canal %u -> %d\n", tempo_ms, canal, adc_raw);
        else log_erro("Falha");
    }
}

// ================== INICIALIZAÇÃO DO ADC CONTÍNUO =========================
static void continuous_adc_init(adc_continuous_handle_t *out_handle) {
    adc_continuous_handle_t handle = NULL;
    // Configuração básica do driver contínuo
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = DMA_BUFFER_SIZE,
        .conv_frame_size = DMA_BUFFER_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &handle));

    // Configuração dos canais e frequência de amostragem
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = HIGH_RATE_TOTAL_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .pattern_num = NUM_CHANNELS,
    };

    // Padrão de amostragem para os 8 canais
    adc_digi_pattern_config_t adc_pattern[NUM_CHANNELS] = {0};
    adc_channel_t channels[NUM_CHANNELS] = {
        ADC1_CH0, ADC1_CH1, ADC1_CH2, ADC1_CH3,
        ADC1_CH4, ADC1_CH5, ADC1_CH6, ADC1_CH7
    };
    for (int i = 0; i < NUM_CHANNELS; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB;
        adc_pattern[i].channel = channels[i];
        adc_pattern[i].unit = ADC_CONTINUOUS_UNIT;
        adc_pattern[i].bit_width = ADC_CONTINUOUS_BITWIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    // Registra o callback que será chamado quando um frame DMA estiver pronto
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = continuous_adc_callback,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));

    *out_handle = handle;
}

// =========================== CALLBACK DO ADC (ISR) ========================
// Definição com IRAM_ATTR (executa em IRAM). Protótipo não tem o atributo.
static bool IRAM_ATTR continuous_adc_callback(adc_continuous_handle_t handle,
                                              const adc_continuous_evt_data_t *edata,
                                              void *user_data) {
    BaseType_t mustYield = pdFALSE;
    g_callback_count++;                        // Incrementa contador de callbacks
    if (g_acquisition_task_handle) {
        vTaskNotifyGiveFromISR(g_acquisition_task_handle, &mustYield);
    }
    return (mustYield == pdTRUE);
}

// =========================== TAREFA PRODUTORA (AQUISIÇÃO) =================
// Roda no Core 1. Lê os dados do DMA, preenche blocos e insere no buffer circular.
static void adc_acquisition_task(void *pvParameters) {
    (void)pvParameters;
    // Buffer para leitura dos dados brutos do DMA
    uint8_t *dma_buffer = (uint8_t *)malloc(DMA_BUFFER_SIZE);
    if (!dma_buffer) {
        log_erro("Falha ao alocar buffer DMA");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // Bloco atual sendo preenchido
    amostra_t *current_block_data = (amostra_t *)malloc(BLOCK_BYTES);
    if (!current_block_data) {
        log_erro("Falha ao alocar bloco de dados");
        free(dma_buffer);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    uint32_t current_block_index = 0;

    while (1) {
        // Aguarda notificação do callback (dados disponíveis)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!g_acquisition_running) {
            if (g_stop_request) break;
            continue;
        }

        uint32_t bytes_read = 0;
        esp_err_t ret = adc_continuous_read(g_cont_handle, dma_buffer, DMA_BUFFER_SIZE, &bytes_read, 0);
        if (ret != ESP_OK || bytes_read == 0) continue;

        adc_continuous_data_t *parsed = (adc_continuous_data_t *)dma_buffer;
        uint32_t num_samples = bytes_read / sizeof(adc_continuous_data_t);
        g_total_samples += num_samples;

        for (uint32_t i = 0; i < num_samples; i++) {
            if (!g_acquisition_running) break;
            uint8_t  canal = parsed[i].channel;   // 0..7
            uint16_t valor = parsed[i].raw_data;

            // Timestamp teórico baseado no índice global da amostra
            int idx = current_block_index + (g_block_sequence * BLOCK_SAMPLES);
            uint64_t tempo_us = g_start_time_us + (idx * (1000000ULL / HIGH_RATE_TOTAL_HZ));
            uint32_t tempo_ms = (uint32_t)(tempo_us / 1000);

            // Armazena no bloco corrente
            current_block_data[current_block_index].time_ms   = tempo_ms;
            current_block_data[current_block_index].n_adc     = canal;
            current_block_data[current_block_index].adc_value = valor;
            current_block_index++;

            // Se o bloco encheu, insere no buffer circular
            if (current_block_index >= BLOCK_SAMPLES) {
                block_t *bloco = (block_t *)malloc(sizeof(block_t));
                if (bloco) {
                    bloco->data = current_block_data;
                    bloco->num_samples = BLOCK_SAMPLES;

                    xSemaphoreTake(g_buffer_mutex, portMAX_DELAY);
                    bool inserted = circ_buffer_push(bloco);
                    xSemaphoreGive(g_buffer_mutex);

                    if (!inserted) {
                        log_erro("Buffer circular cheio - perda de dados!");
                        free(bloco);
                    } else {
                        // Notifica a tarefa escritora
                        if (g_writer_task_handle) xTaskNotifyGive(g_writer_task_handle);
                    }

                    // Prepara novo bloco
                    current_block_data = (amostra_t *)malloc(BLOCK_BYTES);
                    if (!current_block_data) {
                        log_erro("Falha ao alocar novo bloco");
                        g_acquisition_running = false;
                        break;
                    }
                    current_block_index = 0;
                    g_block_sequence++;
                } else {
                    log_erro("Falha ao alocar block_t");
                }
            }
        }
    }
    // Trata último bloco parcial (se houver)
    if (current_block_index > 0) {
        block_t *bloco = (block_t *)malloc(sizeof(block_t));
        if (bloco) {
            bloco->data = current_block_data;
            bloco->num_samples = current_block_index;
            xSemaphoreTake(g_buffer_mutex, portMAX_DELAY);
            bool inserted = circ_buffer_push(bloco);
            xSemaphoreGive(g_buffer_mutex);
            if (!inserted) {
                log_erro("Buffer cheio - último bloco perdido");
                free(bloco);
            } else {
                if (g_writer_task_handle) xTaskNotifyGive(g_writer_task_handle);
            }
        } else {
            free(current_block_data);
        }
    } else {
        free(current_block_data);
    }
    free(dma_buffer);
    log_info("Tarefa de aquisição finalizada");
    vTaskDelete(NULL);
}

// =========================== TAREFA CONSUMIDORA (GRAVAÇÃO SD) ============
// Roda no Core 0. Remove blocos do buffer circular e os escreve no SD.
static void sd_writer_task(void *pvParameters) {
    (void)pvParameters;
    uint32_t file_sequence = 0;
    while (1) {
        // Aguarda notificação da produtora
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xSemaphoreTake(g_buffer_mutex, portMAX_DELAY);
        block_t *bloco = circ_buffer_pop();
        xSemaphoreGive(g_buffer_mutex);

        if (bloco != NULL) {
            esp_err_t err;
            do {
                err = escrever_bloco_binario(bloco, file_sequence);
                if (err != ESP_OK) {
                    log_erro("Falha ao gravar bloco, tentando novamente...");
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            } while (err != ESP_OK);
            file_sequence++;
            free(bloco->data);
            free(bloco);
        }

        // Se flag de parada e buffer vazio, encerra tarefa
        if (g_stop_request) {
            xSemaphoreTake(g_buffer_mutex, portMAX_DELAY);
            bool empty = (g_buffer_count == 0);
            xSemaphoreGive(g_buffer_mutex);
            if (empty) {
                log_info("Tarefa de gravação finalizada");
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

// =========================== CONTROLE DA COLETA ===========================
// Inicia a coleta contínua: monta SD, inicializa ADC, reseta flags.
static void start_continuous_acquisition(void) {
    if (g_acquisition_running) {
        uart_printf("Coleta já em andamento.\n");
        return;
    }
    // Monta SD (se falhar, aborta)
    sdmmc_card_t *card = inicializar_sd_card();
    if (card == NULL) {
        log_erro("Cartão SD não disponível. Abortando.");
        return;
    }
    // Inicializa ADC contínuo se ainda não estiver
    if (g_cont_handle == NULL) {
        continuous_adc_init(&g_cont_handle);
    }
    // Reseta variáveis de controle
    g_buffer_head = g_buffer_tail = g_buffer_count = 0;
    g_block_sequence = 0;
    g_callback_count = 0;
    g_total_samples = 0;
    g_stop_request = false;
    g_acquisition_running = true;
    g_start_time_us = esp_timer_get_time();
    ESP_ERROR_CHECK(adc_continuous_start(g_cont_handle));
    uart_printf("\n=== COLETA CONTÍNUA INICIADA ===\n");
    uart_printf("Pressione '6' no menu para parar.\n");
}

// Para a coleta de forma ordenada.
static void stop_continuous_acquisition(void) {
    if (!g_acquisition_running) {
        uart_printf("Nenhuma coleta em andamento.\n");
        return;
    }
    g_acquisition_running = false;
    g_stop_request = true;
    int timeout = 10; // segundos
    while (g_acquisition_task_handle != NULL || g_writer_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout--;
        if (timeout == 0) {
            log_erro("Timeout aguardando parada das tarefas");
            break;
        }
    }
    adc_continuous_stop(g_cont_handle);

    // Exibe estatísticas finais
    uint32_t total_amostras = g_total_samples;
    float tempo_real = (esp_timer_get_time() - g_start_time_us) / 1000000.0f;
    uart_printf("\n=== COLETA FINALIZADA ===\n");
    uart_printf("Tempo total: %.2f s\n", tempo_real);
    uart_printf("Total de amostras brutas: %lu\n", total_amostras);
    uart_printf("Blocos gravados: %lu\n", g_block_sequence);
    uart_printf("Callback count: %lu\n", g_callback_count);
    uart_printf("Taxa efetiva: %.1f amostras/s\n", total_amostras / tempo_real);
    mostrar_memoria_livre("Após coleta");
}

// =========================== MENU =========================================
static void imprimir_menu(void) {
    uart_printf("\n========================================\n");
    uart_printf("MENU PRINCIPAL\n");
    uart_printf("1 - Teste de entrada e saida (serial)\n");
    uart_printf("2 - Leitura de 1 porta analogica\n");
    uart_printf("3 - Leitura de 2 portas analogicas\n");
    uart_printf("4 - Amostragem periodica de 1 porta\n");
    uart_printf("5 - Iniciar coleta continua (double buffer)\n");
    uart_printf("6 - Parar coleta continua\n");
    uart_printf("0 - Reimprimir menu\n");
    uart_printf("========================================\n");
    uart_printf("Escolha uma opcao: ");
}

// Tarefa que gerencia o menu no Core 0
static void comunicacao_task(void *pvParameters) {
    (void)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_printf("\n===== APLICACAO INICIADA (COLETA CONTÍNUA) =====\n");
    mostrar_memoria_livre("Inicio");
    while (1) {
        imprimir_menu();
        char buffer[32];
        if (uart_gets(buffer, sizeof(buffer)) < 0) {
            log_erro("Timeout, repetindo menu...");
            continue;
        }
        int opcao = atoi(buffer);
        switch (opcao) {
            case 1: teste_entrada_saida_serial(); break;
            case 2: teste_leitura_adc_simples(); break;
            case 3: teste_leitura_adc_duplo(); break;
            case 4: teste_amostragem_periodica_1_canal(2, 0); break;
            case 5: start_continuous_acquisition(); break;
            case 6: stop_continuous_acquisition(); break;
            case 0: break;
            default: uart_printf("Opcao invalida.\n");
        }
    }
}

// =========================== APP MAIN =====================================
void app_main(void) {
    // Configuração da UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 0, NULL, 0));

    // Inicialização do ADC oneshot (para testes básicos)
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &g_oneshot_handle));
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_ONESHOT_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_oneshot_handle, ADC1_CHAN0, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_oneshot_handle, ADC1_CHAN1, &chan_config));

    // Mutex para buffer circular
    g_buffer_mutex = xSemaphoreCreateMutex();
    if (g_buffer_mutex == NULL) {
        log_erro("Falha ao criar mutex");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Criação das tarefas
    // Core 0: comunicação e gravação SD (prioridade 10)
    xTaskCreatePinnedToCore(comunicacao_task, "comunicacao", 8192, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(sd_writer_task,   "sd_writer",   8192, NULL, 10, &g_writer_task_handle, 0);
    // Core 1: aquisição de alta taxa (prioridade 12, maior que as outras)
    xTaskCreatePinnedToCore(adc_acquisition_task, "adc_acq", 16384, NULL, 12, &g_acquisition_task_handle, 1);

    vTaskDelete(NULL);  // app_main pode terminar; as tasks continuam
}