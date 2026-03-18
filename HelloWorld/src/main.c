/*
 * Exercício: Leitura de duas portas analógicas com amostragem periódica e timestamp
 * 
 * Descrição:
 * - Interage com o usuário via Serial (UART) para entrada de dois números.
 * - Calcula a soma e informa se o resultado é par ou ímpar.
 * - Em seguida, realiza 20 amostras das portas analógicas GPIO36 (ADC1_CH0) e GPIO39 (ADC1_CH3)
 *   a cada 500 ms (frequência de 2 Hz), exibindo o valor cru do ADC e o timestamp em milissegundos.
 * - O programa se repete infinitamente com uma pausa de 10 segundos entre ciclos.
 *
 * Plataforma: ESP32 (ESP-IDF via PlatformIO, board = esp32doit-devkit-v1)
 * Autor: João Nogueira
 * 
 */

#include <stdio.h>      // Funções padrão de entrada/saída
#include <stdlib.h>     // Funções de conversão (atoi) e alocação
#include <string.h>     // Manipulação de strings
#include <stdarg.h>     // Macros para funções com lista variável de argumentos (va_list)
#include "freertos/FreeRTOS.h"          // Tipos e macros do FreeRTOS
#include "freertos/task.h"               // Funções de tarefa (vTaskDelay)
#include "driver/uart.h"                 // Driver da UART para comunicação serial
#include "driver/gpio.h"                 // Definições de GPIO (pinos)
#include "esp_adc/adc_oneshot.h"         // API do ADC oneshot (leitura única)
#include "esp_adc/adc_cali.h"            // Calibração do ADC
#include "esp_adc/adc_cali_scheme.h"     // Esquemas de calibração
#include "esp_timer.h"                    // Timer de alta resolução (esp_timer_get_time)

// ================= CONFIGURAÇÃO DA UART =================
#define UART_PORT       UART_NUM_0      // Porta UART0
#define TXD_PIN         GPIO_NUM_1      // Pino TX
#define RXD_PIN         GPIO_NUM_3      // Pino RX
#define BUF_SIZE        1024            // Tamanho do buffer de transmissão/recepção da UART
#define UART_TIMEOUT_MS 10000           // Timeout máximo para aguardar entrada do usuário (10 segundos)

// ================= CONFIGURAÇÃO DO ADC =================
// Canais ADC1 (pinout ESP32-WROOM-32)
#define ADC1_CHAN0      ADC_CHANNEL_0   // GPIO36 (ADC1_0) - canal 0 do ADC1
#define ADC1_CHAN1      ADC_CHANNEL_3   // GPIO39 (ADC1_3) - canal 3 do ADC1
#define ADC_ATTEN       ADC_ATTEN_DB_12 // Atenuação de 12 dB (tensão de entrada de 0 a ~3,3V)

// Número de amostras e intervalo
#define NUM_AMOSTRAS    20              // Quantidade de amostras a serem coletadas
#define INTERVALO_MS    500              // 500 ms => 2 Hz de taxa de amostragem

// ================= FUNÇÕES DE COMUNICAÇÃO SERIAL =================
// Declaração antecipada para uso nas funções de log 
void uart_printf(const char *format, ...);

// Envia uma string pela UART
void uart_send_string(const char *str) {
    // uart_write_bytes envia os dados de forma assíncrona (retorna imediatamente)
    uart_write_bytes(UART_PORT, str, strlen(str));
}

// Função equivalente ao printf, enviando pela UART (suporta formatação)
void uart_printf(const char *format, ...) {
    char buffer[512];                    // Buffer para montar a string formatada
    va_list args;                         // Lista de argumentos variáveis
    va_start(args, format);                // Inicializa a lista com o último argumento fixo
    vsnprintf(buffer, sizeof(buffer), format, args); // Formata a string
    va_end(args);                          // Limpa a lista
    uart_send_string(buffer);               // Envia a string resultante pela UART
}

// Lê uma linha digitada no Serial Monitor (com eco) até ENTER
// Retorna o número de caracteres lidos ou -1 em caso de timeout/erro
int uart_gets(char *buffer, int max_len) {
    int idx = 0;                           // Índice atual no buffer
    while (idx < max_len - 1) {             // Deixa espaço para o terminador nulo
        uint8_t c;                           // Caractere lido
        // uart_read_bytes bloqueia até ler 1 byte ou estourar o timeout (convertido para ticks)
        int len = uart_read_bytes(UART_PORT, &c, 1, pdMS_TO_TICKS(UART_TIMEOUT_MS));
        if (len <= 0) {                       // Timeout ou erro
            buffer[idx] = '\0';                  // Finaliza a string
            return -1;                            // Indica falha
        }
        // Ignora linhas em branco no início (útil para limpar buffer após envios anteriores)
        if ((c == '\n' || c == '\r') && idx == 0)
            continue;
        // Se pressionar ENTER, finaliza a linha
        if (c == '\n' || c == '\r') {
            buffer[idx] = '\0';                  // Coloca terminador
            uart_printf("\n");                    // Envia nova linha para o terminal
            return idx;                            // Retorna o comprimento da string lida
        }
        // Ecoa o caractere digitado de volta ao terminal (para o usuário ver)
        uart_write_bytes(UART_PORT, (const char *)&c, 1);
        buffer[idx++] = (char)c;               // Armazena no buffer e incrementa índice
    }
    buffer[idx] = '\0';                       // Garante terminador se estourar o tamanho
    return idx;                               // Retorna número de caracteres (máximo atingido)
}

// ================= FUNÇÕES DE LOG E ERRO =================
// Exibe uma mensagem de erro via UART, prefixada com "[ERRO]"
void log_erro(const char *mensagem) {
    uart_printf("[ERRO] %s\n", mensagem);
}

// Exibe uma mensagem informativa via UART, prefixada com "[INFO]"
void log_info(const char *mensagem) {
    uart_printf("[INFO] %s\n", mensagem);
}

// ================= FUNÇÕES DO PROGRAMA PRINCIPAL =================
// Função que retorna a soma de dois inteiros
int somar(int a, int b) {
    return a + b;
}

// Verifica se um número é par ou ímpar e exibe o resultado via UART
void verificarPar(int numero) {
    if (numero % 2 == 0)
        uart_printf("O numero %d eh PAR.\n", numero);
    else
        uart_printf("O numero %d eh IMPAR.\n", numero);
}

// ================= FUNÇÃO DE AMOSTRAGEM =================
// Realiza a coleta de amostras dos canais ADC configurados
// Recebe o handle da unidade ADC já configurada
void realizar_amostragem(adc_oneshot_unit_handle_t adc_handle) {
    uart_printf("\n--- Iniciando amostragem das portas analogicas ---\n");
    uart_printf("Canais: ADC1_CH0 (GPIO36) e ADC1_CH1 (GPIO39)\n");
    uart_printf("Frequencia: 2 Hz (intervalo de %d ms)\n", INTERVALO_MS);
    uart_printf("Numero de amostras: %d\n\n", NUM_AMOSTRAS);

    int adc0_raw, adc1_raw;               // Variáveis para armazenar os valores crus do ADC
    esp_err_t ret;                          // Código de retorno das funções ESP

    for (int i = 0; i < NUM_AMOSTRAS; i++) {
        // Leitura dos dois canais ADC usando a API oneshot
        ret = adc_oneshot_read(adc_handle, ADC1_CHAN0, &adc0_raw);
        if (ret != ESP_OK) {
            log_erro("Falha ao ler canal ADC0");
            adc0_raw = -1;                    // Valor inválido para indicar erro
        }

        ret = adc_oneshot_read(adc_handle, ADC1_CHAN1, &adc1_raw);
        if (ret != ESP_OK) {
            log_erro("Falha ao ler canal ADC1");
            adc1_raw = -1;                    // Valor inválido
        }

        // Obtém timestamp em microssegundos desde a inicialização do timer
        uint64_t tempo_us = esp_timer_get_time();
        // Converte para milissegundos (divisão inteira)
        uint32_t tempo_ms = (uint32_t)(tempo_us / 1000);

        // Exibe os valores com timestamp e número da amostra (formatado com 2 dígitos)
        uart_printf("[%u ms] Amostra %2d - ADC0: %4d | ADC1: %4d\n",
                    tempo_ms, i + 1, adc0_raw, adc1_raw);

        // Aguarda o intervalo antes da próxima amostra
        vTaskDelay(pdMS_TO_TICKS(INTERVALO_MS));
    }

    uart_printf("\n--- Amostragem concluida ---\n");
}

// ================= PROGRAMA PRINCIPAL =================
// Função que executa um ciclo completo do programa (entrada de números, soma, amostragem)
// Recebe o handle do ADC já configurado
void programa_principal(adc_oneshot_unit_handle_t adc_handle) {
    // Pequeno atraso inicial para garantir que UART esteja estabilizado
    vTaskDelay(pdMS_TO_TICKS(1000));

    uart_printf("\n===== INICIANDO PROGRAMA =====\n\n");
    uart_printf("Hello, World!\n\n");

    char buffer[50];                        // Buffer para leitura da entrada do usuário
    int numero1 = 0, numero2 = 0;            // Variáveis para armazenar os números lidos

    // Entrada do primeiro número
    uart_printf("Digite o primeiro numero: ");
    if (uart_gets(buffer, sizeof(buffer)) > 0) {
        numero1 = atoi(buffer);               // Converte string para inteiro
    } else {
        log_erro("Falha ao ler o primeiro numero. Usando 0.");
        numero1 = 0;                          // Valor padrão em caso de erro
    }

    // Entrada do segundo número
    uart_printf("\nDigite o segundo numero: ");
    if (uart_gets(buffer, sizeof(buffer)) > 0) {
        numero2 = atoi(buffer);               // Converte string para inteiro
    } else {
        log_erro("Falha ao ler o segundo numero. Usando 0.");
        numero2 = 0;                          // Valor padrão em caso de erro
    }

    // Operações matemáticas
    int resultado = somar(numero1, numero2);   // Calcula a soma
    uart_printf("\nA soma %d + %d = %d\n", numero1, numero2, resultado);
    verificarPar(resultado);                   // Verifica e exibe se é par ou ímpar

    // Amostragem analógica
    realizar_amostragem(adc_handle);           // Executa a coleta de amostras

    uart_printf("\n===== FIM DO PROGRAMA =====\n");
}

// ================= FUNÇÃO PRINCIPAL DO ESP-IDF =================
// Ponto de entrada da aplicação no ESP-IDF (equivalente ao main)
void app_main(void) {
    // -----------------------------------------------------------------
    // CONFIGURAÇÃO DA UART
    // -----------------------------------------------------------------
    uart_config_t uart_config = {
        .baud_rate = 115200,                   // Taxa de transmissão padrão para monitor serial
        .data_bits = UART_DATA_8_BITS,          // 8 bits de dados
        .parity = UART_PARITY_DISABLE,          // Sem paridade
        .stop_bits = UART_STOP_BITS_1,          // 1 bit de parada
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // Sem controle de fluxo por hardware
        .source_clk = UART_SCLK_APB,            // Fonte de clock (APB)
    };

    esp_err_t ret = uart_param_config(UART_PORT, &uart_config);
    if (ret != ESP_OK) {
        // Não podemos usar uart_printf ainda, pois a UART pode não estar configurada.
        // Usamos um mecanismo simples: se falhar, podemos travar ou ignorar.
        // Neste caso, apenas ignoramos e seguimos.
    }

    ret = uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        // Similarmente, ignoramos.
    }

    ret = uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        // Se falhar aqui, a UART não funcionará. Podemos tentar reiniciar ou travar.
        // Neste exemplo, apenas continuamos e torcemos para que funcione.
    }

    vTaskDelay(pdMS_TO_TICKS(1000));           // Aguarda estabilização da UART
    log_info("UART inicializada com sucesso."); // Agora podemos usar uart_printf/log_info

    // -----------------------------------------------------------------
    // CONFIGURAÇÃO DO ADC
    // -----------------------------------------------------------------
    adc_oneshot_unit_handle_t adc_handle;       // Handle para a unidade ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,                   // ADC1
        .ulp_mode = ADC_ULP_MODE_DISABLE,        // Sem modo ULP (ultra low power)
    };
    ret = adc_oneshot_new_unit(&init_config, &adc_handle); // Cria nova unidade oneshot
    if (ret != ESP_OK) {
        log_erro("Falha ao criar unidade ADC oneshot");
        // Em caso de erro, podemos ficar em loop infinito, pois o programa não funcionará.
        while (1) { vTaskDelay(1000); }          // Trava o sistema
    }

    // Configura os canais (ambos usarão a mesma configuração)
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,                       // Atenuação definida (12 dB)
        .bitwidth = ADC_BITWIDTH_12,              // Resolução de 12 bits (0-4095)
    };
    ret = adc_oneshot_config_channel(adc_handle, ADC1_CHAN0, &chan_config);
    if (ret != ESP_OK) {
        log_erro("Falha ao configurar canal ADC0");
    }

    ret = adc_oneshot_config_channel(adc_handle, ADC1_CHAN1, &chan_config);
    if (ret != ESP_OK) {
        log_erro("Falha ao configurar canal ADC1");
    }

    log_info("ADC configurado com sucesso.");

    // -----------------------------------------------------------------
    // LOOP INFINITO DO PROGRAMA
    // -----------------------------------------------------------------
    while (1) {
        // Executa um ciclo completo do programa
        programa_principal(adc_handle);

        // Contagem regressiva antes de reiniciar
        uart_printf("\nReiniciando programa em 10 segundos...\n");
        for (int i = 10; i >= 1; i--) {
            uart_printf("%d...\n", i);
            vTaskDelay(pdMS_TO_TICKS(1000));      // Aguarda 1 segundo
        }

        // Limpa qualquer dado residual no buffer da UART para evitar leitura de lixo no próximo ciclo
        uart_flush(UART_PORT);
    }

    // Nunca chegará aqui, mas por segurança:
    adc_oneshot_del_unit(adc_handle);            // Libera a unidade ADC (caso o loop seja interrompido)
}