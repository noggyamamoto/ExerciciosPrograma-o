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
 * - esp_timer: temporização precisa em microssegundos
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

#include "esp_adc/adc_oneshot.h"   // ADC em modo oneshot
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

// =========================== CONFIGURAÇÃO DO ADC ==========================
#define ADC1_CHAN0              ADC_CHANNEL_0  // GPIO36
#define ADC1_CHAN1              ADC_CHANNEL_3  // GPIO39
#define ADC_ATTEN               ADC_ATTEN_DB_12
#define ADC_BITWIDTH            ADC_BITWIDTH_12

// =========================== CONFIGURAÇÃO DO TESTE =======================
#define HIGH_RATE_HZ            3000          // 3000 amostras por segundo por canal
#define HIGH_RATE_DURATION_S    2             // Duração do teste em segundos
#define NUM_CHANNELS            2             // Dois canais analógicos
#define HIGH_RATE_PERIOD_US     (1000000UL / HIGH_RATE_HZ)   // 333 us
#define NUM_TICKS_TOTAL         (HIGH_RATE_HZ * HIGH_RATE_DURATION_S)
#define NUM_AMOSTRAS_TOTAL      (NUM_TICKS_TOTAL * NUM_CHANNELS) // 3000*2*2 = 12000

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
static SemaphoreHandle_t g_ram_mutex = NULL;         // Mutex para proteger RAM compartilhada
static adc_oneshot_unit_handle_t g_adc_handle = NULL; // Handle global do ADC
static TaskHandle_t g_acquisition_task_handle = NULL; // Task de aquisição
static esp_timer_handle_t g_high_rate_timer = NULL;   // Timer periódico do teste

static volatile bool g_high_rate_active = false;      // Indica se o teste 3000 Hz está ativo

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
static void high_rate_timer_callback(void *arg);
static void registrar_amostra(uint8_t canal, uint16_t valor, uint32_t time_ms);

static void imprimir_menu(void);
static void comunicacao_task(void *pvParameters);

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
    if (adc_oneshot_read(g_adc_handle, ADC1_CHAN0, &adc_raw) == ESP_OK) {
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
    if (adc_oneshot_read(g_adc_handle, ADC1_CHAN0, &adc0_raw) != ESP_OK) {
        log_erro("Falha ao ler ADC0");
    }

    // Lê o canal 1
    if (adc_oneshot_read(g_adc_handle, ADC1_CHAN1, &adc1_raw) != ESP_OK) {
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
            ret = adc_oneshot_read(g_adc_handle, ADC1_CHAN0, &adc_raw);
        } else {
            ret = adc_oneshot_read(g_adc_handle, ADC1_CHAN1, &adc_raw);
        }

        if (ret == ESP_OK) {
            uart_printf("[%u ms] Canal %u -> %d\n", tempo_ms, canal, adc_raw);
        } else {
            log_erro("Falha na leitura da amostra periódica");
        }
    }

    uart_printf("===== FIM DO TESTE =====\n");
}

// =========================== AMOSTRA AUXILIAR =============================
static void registrar_amostra(uint8_t canal, uint16_t valor, uint32_t time_ms) {
    // Protege a RAM compartilhada enquanto escreve no buffer
    if (amostras_coletadas < NUM_AMOSTRAS_TOTAL) {
        amostras_buffer[amostras_coletadas].time_ms = time_ms;
        amostras_buffer[amostras_coletadas].n_adc = canal;
        amostras_buffer[amostras_coletadas].adc_value = valor;
        amostras_coletadas++;
    }

    // Se o buffer encheu, sinaliza término
    if (amostras_coletadas >= NUM_AMOSTRAS_TOTAL) {
        coleta_finalizada = true;
    }
}

// =========================== CALLBACK DO TIMER ============================
static void high_rate_timer_callback(void *arg) {
    (void)arg; // Parâmetro não utilizado

    // Só notifica enquanto o teste de alta taxa estiver ativo
    if (g_high_rate_active && g_acquisition_task_handle != NULL) {
        xTaskNotifyGive(g_acquisition_task_handle); // Acorda a tarefa de aquisição
    }
}

// =========================== TAREFA DE AQUISIÇÃO ==========================
static void adc_acquisition_task(void *pvParameters) {
    (void)pvParameters; // Parâmetro não utilizado

    while (1) {
        // Espera uma notificação do timer
        uint32_t notifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Se o teste não estiver ativo, continua esperando
        if (!g_high_rate_active) {
            continue;
        }

        // Processa todas as notificações acumuladas
        while (notifications-- > 0 && g_high_rate_active && !coleta_finalizada) {
            int adc0_raw = 0;
            int adc1_raw = 0;

            // Timestamp da amostra
            uint64_t tempo_us = esp_timer_get_time();
            uint32_t tempo_ms = (uint32_t)(tempo_us / 1000ULL);

            // Lê os dois canais
            esp_err_t ret0 = adc_oneshot_read(g_adc_handle, ADC1_CHAN0, &adc0_raw);
            esp_err_t ret1 = adc_oneshot_read(g_adc_handle, ADC1_CHAN1, &adc1_raw);

            // Se as leituras deram certo, grava no buffer
            if (ret0 == ESP_OK && ret1 == ESP_OK) {
                // Protege o acesso ao buffer e ao contador
                xSemaphoreTake(g_ram_mutex, portMAX_DELAY);

                registrar_amostra(0, (uint16_t)adc0_raw, tempo_ms);
                registrar_amostra(1, (uint16_t)adc1_raw, tempo_ms);

                xSemaphoreGive(g_ram_mutex);
            }
        }
    }
}

// =========================== TESTE DE ALTA TAXA + CSV ====================
static void testar_alta_taxa_e_salvar_csv(void) {
    uart_printf("\n===== INICIANDO TESTE DE ALTA TAXA (3000 Hz) =====\n");
    uart_printf("Frequencia por canal: %d Hz\n", HIGH_RATE_HZ);
    uart_printf("Intervalo: %lu us\n", (unsigned long)HIGH_RATE_PERIOD_US);
    uart_printf("Duracao: %d s\n", HIGH_RATE_DURATION_S);
    uart_printf("Canais: %d\n", NUM_CHANNELS);
    uart_printf("Total esperado de registros: %d\n", NUM_AMOSTRAS_TOTAL);

    mostrar_memoria_livre("Antes da coleta");

    // Reinicia os contadores e flags
    xSemaphoreTake(g_ram_mutex, portMAX_DELAY);
    amostras_coletadas = 0;
    coleta_finalizada = false;
    xSemaphoreGive(g_ram_mutex);

    g_high_rate_active = true;

    // Cria o timer periódico do teste
    esp_timer_create_args_t timer_args = {
        .callback = high_rate_timer_callback,
        .arg = NULL,
        .name = "high_rate_timer",
    };

    esp_err_t ret = esp_timer_create(&timer_args, &g_high_rate_timer);
    if (ret != ESP_OK) {
        log_erro("Falha ao criar esp_timer do teste de alta taxa");
        g_high_rate_active = false;
        return;
    }

    // Marca o início
    uint64_t tempo_inicio = esp_timer_get_time();

    // Inicia o timer periódico
    ret = esp_timer_start_periodic(g_high_rate_timer, HIGH_RATE_PERIOD_US);
    if (ret != ESP_OK) {
        log_erro("Falha ao iniciar esp_timer periódico");
        esp_timer_delete(g_high_rate_timer);
        g_high_rate_timer = NULL;
        g_high_rate_active = false;
        return;
    }

    // Espera até terminar por buffer cheio ou por tempo máximo
    uint64_t tempo_limite = tempo_inicio + ((uint64_t)HIGH_RATE_DURATION_S * 1000000ULL);

    while (!coleta_finalizada && esp_timer_get_time() < tempo_limite) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Encerra a geração de notificações
    g_high_rate_active = false;
    esp_timer_stop(g_high_rate_timer);

    // Dá um pequeno tempo para a tarefa de aquisição esvaziar o que estiver pendente
    vTaskDelay(pdMS_TO_TICKS(20));

    // Para e remove o timer
    esp_timer_delete(g_high_rate_timer);
    g_high_rate_timer = NULL;

    // Marca o fim
    uint64_t tempo_fim = esp_timer_get_time();
    float tempo_real_segundos = (tempo_fim - tempo_inicio) / 1000000.0f;

    // Quantidade esperada de registros
    int amostras_esperadas = HIGH_RATE_HZ * HIGH_RATE_DURATION_S * NUM_CHANNELS;

    // Quantidade real
    int amostras_reais = amostras_coletadas;

    // Calcula perda
    float perda_percentual = 0.0f;
    if (amostras_esperadas > 0) {
        perda_percentual = 100.0f * (amostras_esperadas - amostras_reais) / amostras_esperadas;
        if (perda_percentual < 0.0f) {
            perda_percentual = 0.0f;
        }
    }

    uart_printf("\n===== RESULTADOS DO TESTE =====\n");
    uart_printf("Tempo real de coleta: %.3f s\n", tempo_real_segundos);
    uart_printf("Amostras esperadas (registros): %d\n", amostras_esperadas);
    uart_printf("Amostras reais coletadas: %d\n", amostras_reais);
    uart_printf("Perda: %.2f%%\n", perda_percentual);
    uart_printf("Taxa efetiva por canal: %.1f Hz\n", (amostras_reais / (float)NUM_CHANNELS) / tempo_real_segundos);

    mostrar_memoria_livre("Após a coleta");

    // Tenta montar o SD e salvar o CSV
    log_info("Tentando salvar os dados no SD card...");
    sdmmc_card_t *card = inicializar_sd_card();

    if (card != NULL) {
        // Substituição temporária por um nome fixo para facilitar testes iniciais
        const char *caminho_teste = MOUNT_POINT "/teste.csv";
        uart_printf("Testando com caminho fixo: '%s'\n", caminho_teste);
        if (escrever_csv(caminho_teste) == ESP_OK) {
            uart_printf("Arquivo fixo salvo com sucesso!\n");
        } else {
            uart_printf("Falha também com caminho fixo.\n");
        }

        //Gera um nome de arquivo com data e hora
        //char caminho_csv[128];
        //time_t agora = time(NULL);
        //struct tm tm_info;
        //localtime_r(&agora, &tm_info);

        //strftime(
            //caminho_csv,
            //sizeof(caminho_csv),
            //MOUNT_POINT "/amostras_%Y%m%d_%H%M%S.csv",
            //&tm_info
        //);
        // Escreve o CSV
        //if (escrever_csv(caminho_csv) == ESP_OK) {
        //    //uart_printf("Arquivo CSV salvo: %s\n", caminho_csv);
        //} else {
        //    //log_erro("Falha ao salvar o arquivo CSV");
        //}

        // Desmonta o cartão e libera o barramento SPI
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        spi_bus_free(SPI3_HOST);
    } else {
        log_erro("Cartao SD nao disponivel. Os dados nao foram salvos.");

        // Mostra as primeiras amostras para depuração
        uart_printf("\nPrimeiras 10 amostras:\n");

        int total_local = amostras_coletadas;
        for (int i = 0; i < 10 && i < total_local; i++) {
            uart_printf(
                "[%lu ms] n_adc=%u, adc_value=%u\n",
                (unsigned long)amostras_buffer[i].time_ms,
                amostras_buffer[i].n_adc,
                amostras_buffer[i].adc_value
            );
        }
    }

    uart_printf("\n===== FIM DO TESTE =====\n");
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

    // ================= ADC =================
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &g_adc_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_handle, ADC1_CHAN0, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_handle, ADC1_CHAN1, &chan_config));

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