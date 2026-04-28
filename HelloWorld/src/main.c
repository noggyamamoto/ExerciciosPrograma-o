/*
 * ============================================================================
 * Projeto integrado para ESP32 (ESP-IDF via PlatformIO)
 * Plataforma: esp32doit-devkit-v1
 *
 * O código abaixo reúne:
 * 1) Entrada e saída de dados pela UART (Serial Monitor)
 * 2) Teste para confirmar que o programa roda no microcontrolador
 * 3) Leitura e manipulação de uma porta analógica
 * 4) Leitura e manipulação de duas portas analógicas
 * 5) Amostragem de uma porta em frequência definida, com timestamp
 * 6) Teste de alta taxa de amostragem (3000 Hz) com quantificação de perda
 * 7) Escrita dos dados em cartão SD no formato CSV
 *
 * Estrutura geral:
 * - Core 0: comunicação / menu / interação com o usuário
 * - Core 1: aquisição em alta taxa
 * - Mutex: proteção da RAM compartilhada
 * - ADC contínuo com DMA para alta taxa
 * ============================================================================ 
 */

// =========================== BIBLIOTECAS PADRÃO ===========================
#include <stdio.h>      // Funções de entrada e saída padrão
#include <stdlib.h>     // Funções utilitárias como atoi
#include <string.h>     // Manipulação de strings
#include <stdarg.h>     // Argumentos variáveis
#include <time.h>       // Data e hora para nomear arquivos no SD 
#include <stdbool.h>    // Tipo bool 
#include <errno.h>      // Códigos de erro padrão

// =========================== BIBLIOTECAS DO FREERTOS ======================
#include "freertos/FreeRTOS.h"     // Kernel do FreeRTOS
#include "freertos/task.h"         // Tarefas do FreeRTOS
#include "freertos/semphr.h"       // Semáforos / mutex

// =========================== BIBLIOTECAS DO ESP-IDF =======================
#include "driver/uart.h"           // Driver UART
#include "driver/gpio.h"           // Controle de GPIO
#include "driver/spi_master.h"     // SPI master
#include "esp_timer.h"             // Timer de alta resolução
#include "esp_heap_caps.h"         // Memória livre

#include "esp_adc/adc_oneshot.h"   // ADC oneshot para as tarefas 2, 3 e 4
#include "esp_adc/adc_continuous.h" // ADC contínuo para o teste de alta taxa 
#include "driver/sdspi_host.h"     // SD via SPI
#include "sdmmc_cmd.h"             // Estruturas do cartão SD
#include "esp_vfs_fat.h"           // FAT + VFS

// =========================== CONFIGURAÇÃO DA UART =========================
#define UART_PORT               UART_NUM_0   // UART usada pelo Serial Monitor
#define TXD_PIN                 GPIO_NUM_1   // TX padrão do ESP32
#define RXD_PIN                 GPIO_NUM_3   // RX padrão do ESP32
#define UART_RX_BUF_SIZE        2048         // Buffer RX interno do driver
#define UART_TX_BUF_SIZE        2048         // Buffer TX interno do driver
#define UART_LINE_TIMEOUT_MS    10000        // Timeout para ler uma linha inteira

// =========================== CONFIGURAÇÃO DO ADC ONESHOT ==================
#define ADC1_CHAN0              ADC_CHANNEL_0  // GPIO36
#define ADC1_CHAN1              ADC_CHANNEL_3  // GPIO39
#define ADC_ATTEN               ADC_ATTEN_DB_12
#define ADC_ONESHOT_BITWIDTH    ADC_BITWIDTH_12  

// =========================== CONFIGURAÇÃO DO ADC CONTÍNUO (DMA) ============
#define ADC_CONTINUOUS_UNIT     ADC_UNIT_1
#define ADC_CONTINUOUS_CHAN0    ADC_CHANNEL_0   // GPIO36
#define ADC_CONTINUOUS_CHAN1    ADC_CHANNEL_3   // GPIO39
#define ADC_ATTEN_DB            ADC_ATTEN_DB_12
#define ADC_CONTINUOUS_BITWIDTH SOC_ADC_DIGI_MAX_BITWIDTH 

// =========================== CONFIGURAÇÃO DO TESTE DE ALTA TAXA ===========
// Frequência total de amostragem (soma dos dois canais)
// 3000 amostras por segundo por canal -> 6000 amostras/seg total
#define HIGH_RATE_TOTAL_HZ      6000
#define HIGH_RATE_DURATION_S    2
#define NUM_CHANNELS            2
#define NUM_AMOSTRAS_TOTAL      (HIGH_RATE_TOTAL_HZ * HIGH_RATE_DURATION_S)  // 12000

// Configuração do buffer DMA (tamanho em bytes)
#define DMA_BUFFER_SIZE         4096    // suficiente para centenas de amostras

// =========================== CONFIGURAÇÃO DO SD CARD ======================
#define PIN_NUM_MISO            19
#define PIN_NUM_MOSI            23
#define PIN_NUM_CLK             18
#define PIN_NUM_CS              5
#define MOUNT_POINT             "/sdcard"

// =========================== ESTRUTURA DAS AMOSTRAS =======================
typedef struct {
    uint32_t time_ms;    // Timestamp em milissegundos
    uint8_t n_adc;       // Número do canal (0 ou 1)
    uint16_t adc_value;  // Valor bruto do ADC
} amostra_t;

// =========================== BUFFER DE AMOSTRAS ===========================
static amostra_t amostras_buffer[NUM_AMOSTRAS_TOTAL]; // Buffer estático em RAM
static volatile int amostras_coletadas = 0;           // Quantidade de registros gravados
static volatile bool coleta_finalizada = false;       // Flag de fim da coleta

// =========================== RECURSOS COMPARTILHADOS ======================
static SemaphoreHandle_t g_ram_mutex = NULL;         // Protege o buffer
static adc_oneshot_unit_handle_t g_oneshot_handle = NULL;  // Para testes básicos
static adc_continuous_handle_t g_cont_handle = NULL;      // Para alta taxa (DMA)
static TaskHandle_t g_acquisition_task_handle = NULL;

static volatile bool g_high_rate_active = false;      // Controla a coleta ativa
static uint64_t g_start_time_us = 0;                  // Início da coleta

// =========================== PROTÓTIPOS ===================================
static void uart_send_string(const char *str);
static void uart_printf(const char *format, ...);
static int uart_gets(char *buffer, int max_len);
static void log_erro(const char *mensagem);
static void log_info(const char *mensagem);
static void mostrar_memoria_livre(const char *mensagem);
static int somar(int a, int b);
static void verificarPar(int numero);
static sdmmc_card_t *inicializar_sd_card(void);
static esp_err_t escrever_csv(const char *caminho_arquivo);
static void teste_entrada_saida_serial(void);
static void teste_leitura_adc_simples(void);
static void teste_leitura_adc_duplo(void);
static void teste_amostragem_periodica_1_canal(uint32_t frequencia_hz, uint8_t canal);
static void testar_alta_taxa_e_salvar_csv(void);
static void adc_acquisition_task(void *pvParameters);
// Protótipo do callback sem IRAM_ATTR (o atributo fica apenas na definição)
static bool continuous_adc_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
static void registrar_amostra(uint8_t canal, uint16_t valor, uint32_t time_ms);
static void imprimir_menu(void);
static void comunicacao_task(void *pvParameters);
static void continuous_adc_init(adc_continuous_handle_t *out_handle);

// =========================== UART: ENVIO ==================================
static void uart_send_string(const char *str) {
    // Envia uma string completa pela UART0
    uart_write_bytes(UART_PORT, str, strlen(str));
}

static void uart_printf(const char *format, ...) {
    // Buffer temporário para montar a string formatada
    char buffer[512];

    // Inicia a leitura dos argumentos variáveis
    va_list args;
    va_start(args, format);

    // Formata a string no buffer
    vsnprintf(buffer, sizeof(buffer), format, args);

    // Finaliza o uso dos argumentos variáveis
    va_end(args);

    // Envia a string formatada pela UART
    uart_send_string(buffer);
}

// =========================== UART: LEITURA DE LINHA =======================
static int uart_gets(char *buffer, int max_len) {
    int idx = 0; // Índice de escrita no buffer

    while (idx < max_len - 1) {
        uint8_t c; // Byte lido da UART

        // Lê um byte por vez com timeout
        int len = uart_read_bytes(
            UART_PORT,
            &c,
            1,
            pdMS_TO_TICKS(UART_LINE_TIMEOUT_MS)
        );

        // Se estourar o timeout, encerra a leitura
        if (len <= 0) {
            buffer[idx] = '\0';
            return -1;
        }

        // Ignora ENTER no começo da linha
        if ((c == '\n' || c == '\r') && idx == 0) {
            continue;
        }

        // Se encontrou ENTER, finaliza a string
        if (c == '\n' || c == '\r') {
            buffer[idx] = '\0';
            uart_printf("\n");
            return idx;
        }

        // Ecoa o caractere digitado no monitor serial
        uart_write_bytes(UART_PORT, (const char *)&c, 1);

        // Armazena o caractere no buffer local
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
static int somar(int a, int b) {
    return a + b;
}

static void verificarPar(int numero) {
    if (numero % 2 == 0) {
        uart_printf("O numero %d e PAR.\n", numero);
    } else {
        uart_printf("O numero %d e IMPAR.\n", numero);
    }
}

// =========================== SD CARD ======================================
static sdmmc_card_t *inicializar_sd_card(void) {
    esp_err_t ret;                 // Guarda retorno das funções
    sdmmc_card_t *card = NULL;     // Ponteiro do cartão SD montado

    // Configura o host SPI para o SD
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST; // SPI3 no ESP32

    host.max_freq_khz = 400;

    // Configura o barramento SPI físico
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    // Inicializa o barramento SPI com DMA automático
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        log_erro("Falha ao inicializar barramento SPI para SD");
        return NULL;
    }

    // Configuração do cartão SD no modo SPI
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    // Configuração do mount FAT
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    // Monta o cartão SD
    ret = esp_vfs_fat_sdspi_mount(
        MOUNT_POINT,
        &host,
        &slot_config,
        &mount_config,
        &card
    );

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            log_erro("Falha ao montar o sistema de arquivos FAT.");
        } else {
            log_erro("Falha ao inicializar o cartao SD.");
        }
        spi_bus_free(host.slot);
        return NULL;
    }

    log_info("Cartao SD inicializado com sucesso!");
    uart_printf(
        "Capacidade aproximada: %llu MB\n",
        ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024ULL * 1024ULL)
    );

    return card;
}

static esp_err_t escrever_csv(const char *caminho_arquivo) {
    // Abre o arquivo CSV para escrita
    FILE *f = fopen(caminho_arquivo, "w");
    if (f == NULL) {
        //Mostrar se o erro é, ENOENT (diretório não encontrado), 
        //ENOSPC (sem espaço), EROFS (somente leitura) ou EINVAL.
        uart_printf("[ERRO] fopen falhou: %s (errno=%d)\n", strerror(errno), errno);
        log_erro("Falha ao criar arquivo CSV");
        return ESP_FAIL;
    }

    // Usa buffer de I/O para reduzir overhead de escrita
    static char io_buffer[4096];
    setvbuf(f, io_buffer, _IOFBF, sizeof(io_buffer));

    // Cabeçalho do CSV
    fprintf(f, "time_ms,n_adc,adc_value\n");    //  Como escrever três colunas respectivamente
                                                //  time_ms,n_adc,adc_value?
    // Copia a quantidade total para uma variável local
    int total = amostras_coletadas;

    // Percorre o buffer e grava cada linha do CSV  || // Como escrever os valores nas linhas e colunas do CSV,
    for (int i = 0; i < total; i++) {                       // campos time_ms, n_adc e adc_value?
        fprintf(
            f,                                    
            "%lu,%u,%u\n",                       
            (unsigned long)amostras_buffer[i].time_ms,
            amostras_buffer[i].n_adc,
            amostras_buffer[i].adc_value
        );
    }

    // Fecha o arquivo
    fclose(f);

    log_info("Arquivo CSV salvo com sucesso!");
    return ESP_OK;
}

// =========================== TESTE: ENTRADA E SAÍDA =======================
static void teste_entrada_saida_serial(void) {
    char buffer[64];

    uart_printf("\n===== TESTE DE ENTRADA E SAIDA =====\n");
    uart_printf("Digite o primeiro numero: ");

    int numero1 = 0;
    if (uart_gets(buffer, sizeof(buffer)) > 0) {
        numero1 = atoi(buffer);
    } else {
        log_erro("Timeout na leitura do primeiro numero. Usando 0.");
    }

    uart_printf("Digite o segundo numero: ");

    int numero2 = 0;
    if (uart_gets(buffer, sizeof(buffer)) > 0) {
        numero2 = atoi(buffer);
    } else {
        log_erro("Timeout na leitura do segundo numero. Usando 0.");
    }

    int resultado = somar(numero1, numero2);

    uart_printf("\nResultado da soma: %d + %d = %d\n", numero1, numero2, resultado);
    verificarPar(resultado);

    uart_printf("===== FIM DO TESTE =====\n");
}

// =========================== TESTE: 1 PORTA ANALOGICA =====================
static void teste_leitura_adc_simples(void) {
    uart_printf("\n===== LEITURA DE 1 PORTA ANALOGICA =====\n");

    int adc_raw = 0;

    // Lê o canal ADC1_CHAN0
    if (adc_oneshot_read(g_oneshot_handle, ADC1_CHAN0, &adc_raw) == ESP_OK) {
        uart_printf("ADC0 (GPIO36) = %d\n", adc_raw);
    } else {
        log_erro("Falha ao ler ADC0");
    }

    uart_printf("===== FIM DO TESTE =====\n");
}

// =========================== TESTE: 2 PORTAS ANALOGICAS ===================
static void teste_leitura_adc_duplo(void) {
    uart_printf("\n===== LEITURA DE 2 PORTAS ANALOGICAS =====\n");

    int adc0_raw = 0;
    int adc1_raw = 0;

    // Lê o canal 0
    if (adc_oneshot_read(g_oneshot_handle, ADC1_CHAN0, &adc0_raw) != ESP_OK) {
        log_erro("Falha ao ler ADC0");
    }

    // Lê o canal 1
    if (adc_oneshot_read(g_oneshot_handle, ADC1_CHAN1, &adc1_raw) != ESP_OK) {
        log_erro("Falha ao ler ADC1");
    }

    uart_printf("ADC0 (GPIO36) = %d\n", adc0_raw);
    uart_printf("ADC1 (GPIO39) = %d\n", adc1_raw);

    uart_printf("===== FIM DO TESTE =====\n");
}

// =========================== TESTE: AMOSTRAGEM PERIÓDICA ==================
static void teste_amostragem_periodica_1_canal(uint32_t frequencia_hz, uint8_t canal) {
    uart_printf("\n===== AMOSTRAGEM PERIODICA DE 1 CANAL =====\n");
    uart_printf("Frequencia escolhida: %u Hz\n", (unsigned int)frequencia_hz);
    uart_printf("Canal escolhido: %u\n", (unsigned int)canal);
    uart_printf("Serão mostradas 5 amostras com timestamp.\n");

    // Calcula o período em microssegundos
    uint64_t periodo_us = 1000000ULL / frequencia_hz;

    // Marca o instante inicial
    uint64_t proximo_instante = esp_timer_get_time();

    for (int i = 0; i < 5; i++) {
        // Calcula o instante da próxima amostra
        proximo_instante += periodo_us;

        // Espera até chegar o instante programado
        while (esp_timer_get_time() < proximo_instante) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        int adc_raw = 0;
        uint64_t tempo_us = esp_timer_get_time();
        uint32_t tempo_ms = (uint32_t)(tempo_us / 1000ULL);

        // Lê o canal pedido
        esp_err_t ret = ESP_FAIL;
        if (canal == 0) {
            ret = adc_oneshot_read(g_oneshot_handle, ADC1_CHAN0, &adc_raw);
        } else {
            ret = adc_oneshot_read(g_oneshot_handle, ADC1_CHAN1, &adc_raw);
        }

        if (ret == ESP_OK) {
            uart_printf("[%u ms] Canal %u -> %d\n", tempo_ms, canal, adc_raw);
        } else {
            log_erro("Falha na leitura da amostra periódica");
        }
    }

    uart_printf("===== FIM DO TESTE =====\n");
}

// ================== INICIALIZAÇÃO DO ADC CONTÍNUO (DMA) ==================
static void continuous_adc_init(adc_continuous_handle_t *out_handle) {
    adc_continuous_handle_t handle = NULL;

    // 1. Configuração básica do driver contínuo
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = DMA_BUFFER_SIZE,   // Tamanho máximo do buffer interno
        .conv_frame_size = DMA_BUFFER_SIZE,      // Cada frame terá esse tamanho (pode ser menor)
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &handle));

    // 2. Configuração dos canais e frequência de amostragem
    //    A frequência total (soma dos canais) será HIGH_RATE_TOTAL_HZ
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = HIGH_RATE_TOTAL_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,     // Usa apenas ADC1
        .pattern_num = NUM_CHANNELS,
    };

    // Padrão de amostragem: alterna entre os dois canais
    adc_digi_pattern_config_t adc_pattern[NUM_CHANNELS] = {0};
    for (int i = 0; i < NUM_CHANNELS; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB;
        adc_pattern[i].channel = (i == 0) ? ADC_CONTINUOUS_CHAN0 : ADC_CONTINUOUS_CHAN1;
        adc_pattern[i].unit = ADC_CONTINUOUS_UNIT;
        adc_pattern[i].bit_width = ADC_CONTINUOUS_BITWIDTH;   // Nome corrigido
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    // 3. Registra o callback que será chamado quando um frame estiver pronto
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = continuous_adc_callback,   // função chamada na ISR
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));

    *out_handle = handle;
}

// =========================== CALLBACK DO ADC CONTÍNUO (ISR) ===============
// Esta função roda em contexto de interrupção. Deve ser rápida e não pode
// chamar funções bloqueantes. Apenas notifica a tarefa de aquisição.
static bool IRAM_ATTR continuous_adc_callback(adc_continuous_handle_t handle,
                                              const adc_continuous_evt_data_t *edata,
                                              void *user_data) {
    BaseType_t mustYield = pdFALSE;
    // Notifica a tarefa de aquisição de que há dados disponíveis
    if (g_acquisition_task_handle) {
        vTaskNotifyGiveFromISR(g_acquisition_task_handle, &mustYield);
    }
    return (mustYield == pdTRUE);
}

// =========================== REGISTRAR AMOSTRA NO BUFFER ==================
static void registrar_amostra(uint8_t canal, uint16_t valor, uint32_t time_ms) {
    if (amostras_coletadas < NUM_AMOSTRAS_TOTAL) {
        amostras_buffer[amostras_coletadas].time_ms = time_ms;
        amostras_buffer[amostras_coletadas].n_adc = canal;
        amostras_buffer[amostras_coletadas].adc_value = valor;
        amostras_coletadas++;
    }
    if (amostras_coletadas >= NUM_AMOSTRAS_TOTAL) {
        coleta_finalizada = true;
    }
}

// =========================== TAREFA DE AQUISIÇÃO (CORE 1) =================
// Esta tarefa é responsável por ler os dados do buffer DMA, parseá-los e
// armazená-los no buffer de amostras com timestamps teóricos.
static void adc_acquisition_task(void *pvParameters) {
    (void)pvParameters;
    uint8_t *dma_buffer = (uint8_t *)malloc(DMA_BUFFER_SIZE);
    if (!dma_buffer) {
        log_erro("Falha ao alocar buffer DMA");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    while (1) {
        // Aguarda notificação do callback (novo frame de dados)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!g_high_rate_active) continue;  // Coleta não iniciada ou já terminou

        // Lê os dados disponíveis no driver (não bloqueante, timeout 0)
        uint32_t bytes_read = 0;
        esp_err_t ret = adc_continuous_read(g_cont_handle, dma_buffer, DMA_BUFFER_SIZE, &bytes_read, 0);
        if (ret != ESP_OK || bytes_read == 0) continue;

        // Parse dos dados brutos para estruturas legíveis
        adc_continuous_data_t *parsed = (adc_continuous_data_t *)dma_buffer;
        uint32_t num_samples = bytes_read / sizeof(adc_continuous_data_t);

        // Para cada amostra, calcula o timestamp teórico baseado no início da coleta
        // e no índice global da amostra.
        for (uint32_t i = 0; i < num_samples; i++) {
            if (!g_high_rate_active || coleta_finalizada) break;

            uint8_t canal = parsed[i].channel;  // 0 ou 1 (depende do padrão)
            uint16_t valor = parsed[i].raw_data;
            // Índice global da amostra: total já coletado + i
            int idx = amostras_coletadas + i;
            if (idx >= NUM_AMOSTRAS_TOTAL) break;

            // Tempo teórico em ms desde o início
            uint64_t tempo_us = g_start_time_us + (idx * (1000000ULL / HIGH_RATE_TOTAL_HZ));
            uint32_t tempo_ms = (uint32_t)(tempo_us / 1000);

            // Protege o buffer compartilhado
            xSemaphoreTake(g_ram_mutex, portMAX_DELAY);
            registrar_amostra(canal, valor, tempo_ms);
            xSemaphoreGive(g_ram_mutex);
        }

        // Se o buffer de amostras encheu, finaliza a coleta
        if (coleta_finalizada) {
            g_high_rate_active = false;
        }
    }
    free(dma_buffer);
}

// =========================== TESTE DE ALTA TAXA (DMA) =====================
static void testar_alta_taxa_e_salvar_csv(void) {
    uart_printf("\n===== INICIANDO TESTE DE ALTA TAXA COM DMA =====\n");
    uart_printf("Frequencia total: %d Hz (%d Hz por canal)\n", HIGH_RATE_TOTAL_HZ, HIGH_RATE_TOTAL_HZ/NUM_CHANNELS);
    uart_printf("Duracao: %d s\n", HIGH_RATE_DURATION_S);
    uart_printf("Canais: %d\n", NUM_CHANNELS);
    uart_printf("Total esperado de registros: %d\n", NUM_AMOSTRAS_TOTAL);

    mostrar_memoria_livre("Antes da coleta");

    // Prepara o buffer de amostras
    xSemaphoreTake(g_ram_mutex, portMAX_DELAY);
    amostras_coletadas = 0;
    coleta_finalizada = false;
    xSemaphoreGive(g_ram_mutex);

    // Inicializa o ADC contínuo (se já não estiver)
    if (g_cont_handle == NULL) {
        continuous_adc_init(&g_cont_handle);
    }

    // Marca o início da coleta
    g_start_time_us = esp_timer_get_time();
    g_high_rate_active = true;

    // Inicia a aquisição contínua
    ESP_ERROR_CHECK(adc_continuous_start(g_cont_handle));

    // Aguarda o tempo de coleta ou buffer cheio
    uint64_t tempo_limite = g_start_time_us + (HIGH_RATE_DURATION_S * 1000000ULL);
    while (!coleta_finalizada && esp_timer_get_time() < tempo_limite) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Finaliza a coleta
    g_high_rate_active = false;
    adc_continuous_stop(g_cont_handle);

    uint64_t tempo_fim = esp_timer_get_time();
    float tempo_real_segundos = (tempo_fim - g_start_time_us) / 1000000.0f;

    int amostras_esperadas = NUM_AMOSTRAS_TOTAL;
    int amostras_reais = amostras_coletadas;
    float perda_percentual = 100.0f * (amostras_esperadas - amostras_reais) / amostras_esperadas;
    if (perda_percentual < 0) perda_percentual = 0;

    uart_printf("\n===== RESULTADOS DO TESTE =====\n");
    uart_printf("Tempo real de coleta: %.3f s\n", tempo_real_segundos);
    uart_printf("Amostras esperadas: %d\n", amostras_esperadas);
    uart_printf("Amostras coletadas: %d\n", amostras_reais);
    uart_printf("Perda: %.2f%%\n", perda_percentual);
    uart_printf("Taxa efetiva por canal: %.1f Hz\n", (amostras_reais / (float)NUM_CHANNELS) / tempo_real_segundos);

    mostrar_memoria_livre("Após a coleta");

    // Salva em SD
    log_info("Salvando dados no SD...");
    sdmmc_card_t *card = inicializar_sd_card();
    if (card != NULL) {
        const char *caminho = MOUNT_POINT "/teste_dma.csv";
        if (escrever_csv(caminho) == ESP_OK) {
            uart_printf("Arquivo salvo: %s\n", caminho);
        } else {
            log_erro("Falha ao salvar CSV");
        }
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        spi_bus_free(SPI3_HOST);
    } else {
        log_erro("SD não disponível. Exibindo primeiras 10 amostras:");
        for (int i = 0; i < 10 && i < amostras_reais; i++) {
            uart_printf("[%lu ms] n_adc=%u, valor=%u\n",
                        (unsigned long)amostras_buffer[i].time_ms,
                        amostras_buffer[i].n_adc,
                        amostras_buffer[i].adc_value);
        }
    }
}

// =========================== MENU =========================================
static void imprimir_menu(void) {
    uart_printf("\n========================================\n");
    uart_printf("MENU PRINCIPAL\n");
    uart_printf("1 - Teste de entrada e saida (serial)\n");
    uart_printf("2 - Leitura de 1 porta analogica\n");
    uart_printf("3 - Leitura de 2 portas analogicas\n");
    uart_printf("4 - Amostragem periodica de 1 porta\n");
    uart_printf("5 - Teste de alta taxa (3000 Hz) + CSV no SD\n");
    uart_printf("0 - Reimprimir menu\n");
    uart_printf("========================================\n");
    uart_printf("Escolha uma opcao: ");
}

// =========================== TAREFA DE COMUNICACAO ========================
static void comunicacao_task(void *pvParameters) {
    (void)pvParameters; // Não usado

    // Pequena pausa para estabilizar a UART e o monitor serial
    vTaskDelay(pdMS_TO_TICKS(1000));

    uart_printf("\n===== APLICACAO INICIADA NO ESP32 =====\n");
    mostrar_memoria_livre("Inicio do programa");

    while (1) {
        // Mostra o menu
        imprimir_menu();

        // Lê a opção digitada
        char buffer[32];
        int lidos = uart_gets(buffer, sizeof(buffer));

        if (lidos < 0) {
            log_erro("Timeout ao ler a opcao do menu");
            continue;
        }

        int opcao = atoi(buffer);

        switch (opcao) {
            case 1:
                teste_entrada_saida_serial();
                break;

            case 2:
                teste_leitura_adc_simples();
                break;

            case 3:
                teste_leitura_adc_duplo();
                break;

            case 4:
                teste_amostragem_periodica_1_canal(2, 0); // 2 Hz no canal 0
                break;

            case 5:
                testar_alta_taxa_e_salvar_csv();
                break;

            case 0:
                // Apenas reapresenta o menu
                break;

            default:
                uart_printf("Opcao invalida.\n");
                break;
        }
    }
}

// =========================== APP MAIN =====================================
void app_main(void) {
    // ================= UART =================
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Aplica a configuracao na UART0
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));

    // Define os pinos da UART0
    ESP_ERROR_CHECK(
        uart_set_pin(
            UART_PORT,
            TXD_PIN,
            RXD_PIN,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE
        )
    );

    // Instala o driver UART com buffers internos
    ESP_ERROR_CHECK(
        uart_driver_install(
            UART_PORT,
            UART_RX_BUF_SIZE,
            UART_TX_BUF_SIZE,
            0,
            NULL,
            0
        )
    );

    // ================= ADC ONESHOT =================
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &g_oneshot_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_ONESHOT_BITWIDTH,   // Nome corrigido
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_oneshot_handle, ADC1_CHAN0, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_oneshot_handle, ADC1_CHAN1, &chan_config));

    // ================= MUTEX =================
    g_ram_mutex = xSemaphoreCreateMutex();
    if (g_ram_mutex == NULL) {
        log_erro("Falha ao criar mutex");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // ================= TASKS EM CORES DIFERENTES =================
    // Core 0: comunicação/menu
    xTaskCreatePinnedToCore(
        comunicacao_task,
        "comunicacao_task",
        8192,
        NULL,
        10,
        NULL,
        0
    );

    // Core 1: aquisição para o teste de alta taxa
    xTaskCreatePinnedToCore(
        adc_acquisition_task,
        "adc_acquisition_task",
        4096,
        NULL,
        12,
        &g_acquisition_task_handle,
        1
    );

    // A app_main termina aqui; as tarefas ficam rodando
    vTaskDelete(NULL);
}