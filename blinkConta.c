#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include <stdio.h>
#include "pico/bootrom.h"
#include "ws2818b.pio.h"

//=====  Definindo pinos  =====//

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define LED_R 13
#define LED_B 12
#define LED_G 11
#define botaoA 5
#define botaoB 6
#define buzzer 21
#define LED_PIN 7
#define LED_COUNT 25


//=====  Variáveis Globais  =====//

bool ModoSemaforo = true;
absolute_time_t last_interrupt_time = 0;
volatile bool estadoBotao = true;     // Estado anterior do botão
volatile TickType_t BotaoDebounce = 0; // Para implementar debounce
// Variáveis globais
volatile int estado_semaforo = 0; // 0 = verde, 1 = amarelo, 2 = vermelho
uint sm;                       // Máquina de estado do PIO
bool Noite = false;

//=====  Funções para o Buzzer  =====//

void init_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM); // Configura o GPIO como PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(slice_num, 125.0f);     // Define o divisor do clock para 1 MHz
    pwm_set_wrap(slice_num, 1000);        // Define o TOP para frequência de 1 kHz
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0); // Razão cíclica inicial
    pwm_set_enabled(slice_num, true);     // Habilita o PWM
} void set_buzzer_tone(uint gpio, uint freq) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    uint top = 1000000 / freq;            // Calcula o TOP para a frequência desejada
    pwm_set_wrap(slice_num, top);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), top / 2); // 50% duty cycle
} void stop_buzzer(uint gpio) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0); // Desliga o PWM
}

//=====  Task Responsável pelas cores  =====//

void vTaskLED()
{
    gpio_init(LED_R);
    gpio_init(LED_G);
    gpio_init(LED_B);
    gpio_set_dir(LED_R, GPIO_OUT);
    gpio_set_dir(LED_G, GPIO_OUT);
    gpio_set_dir(LED_B, GPIO_OUT);
    
    while (true)
    {
        if (ModoSemaforo == true && Noite == false)
        {
            // Luz Verde //
            estado_semaforo = 0;
            gpio_put(LED_R , false);
            gpio_put(LED_G, true);
            vTaskDelay(pdMS_TO_TICKS(4000));

            if (ModoSemaforo == true && Noite == false)
            {
                // Luz Amarela //
                estado_semaforo = 1;
                gpio_put(LED_R, true);
                vTaskDelay(pdMS_TO_TICKS(2000));
                gpio_put(LED_R, false);
                gpio_put(LED_G, false);

                if (ModoSemaforo == true && Noite == false)
                {
                    estado_semaforo = 2;
                    // Luz Vermelha //
                    gpio_put(LED_R , true);
                    vTaskDelay(pdMS_TO_TICKS(6000));
                }

            }

        } 
        
        else if (ModoSemaforo == false)
        {
            // Luz Amarela Piscando
            Noite = true;
            gpio_put(LED_R , true);
            gpio_put(LED_G, true);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_put(LED_R , false);
            gpio_put(LED_G, false);
            vTaskDelay(pdMS_TO_TICKS(2000));
            Noite = false;
        }
    }
}

//=====  Task responsável pelo aviso sonoro  =====//

void vTaskBuzzer()
{
    gpio_init(buzzer);
    gpio_set_dir(buzzer, GPIO_OUT);
    init_pwm(buzzer);

    while (true)
    {
        if (ModoSemaforo == true && Noite == false)
        {
            // Luz Verde //
            set_buzzer_tone(buzzer, 330);
            vTaskDelay(pdMS_TO_TICKS(1000));
            stop_buzzer(buzzer);        
            vTaskDelay(pdMS_TO_TICKS(3000));

            if (ModoSemaforo == true && Noite == false)
            {   
                // Luz Amarela //
                set_buzzer_tone(buzzer, 440);
                vTaskDelay(pdMS_TO_TICKS(250));
                stop_buzzer(buzzer);
                vTaskDelay(pdMS_TO_TICKS(250));
                set_buzzer_tone(buzzer, 440);
                vTaskDelay(pdMS_TO_TICKS(250));
                stop_buzzer(buzzer);
                vTaskDelay(pdMS_TO_TICKS(250));
                set_buzzer_tone(buzzer, 440);
                vTaskDelay(pdMS_TO_TICKS(250));
                stop_buzzer(buzzer);
                vTaskDelay(pdMS_TO_TICKS(250));
                set_buzzer_tone(buzzer, 440);
                vTaskDelay(pdMS_TO_TICKS(250));
                stop_buzzer(buzzer);
                vTaskDelay(pdMS_TO_TICKS(250));

                if (ModoSemaforo == true && Noite == false)
                {
                    // Luz Vermelha //
                    set_buzzer_tone(buzzer, 220);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    stop_buzzer(buzzer);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    set_buzzer_tone(buzzer, 220);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    stop_buzzer(buzzer);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    set_buzzer_tone(buzzer, 220);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    stop_buzzer(buzzer);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                }
            }
        } 

        else if (ModoSemaforo == false)
        {
            // Luz Amarela Piscando
            set_buzzer_tone(buzzer, 220);
            vTaskDelay(pdMS_TO_TICKS(500));
            set_buzzer_tone(buzzer, 200);
            vTaskDelay(pdMS_TO_TICKS(500));
            stop_buzzer(buzzer);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }   
    }
}

//=====  Task responsável pela Flag que muda o modo =====//

void vAlternarModo(void *pvParameters) {
    const TickType_t tempoDebounce = pdMS_TO_TICKS(200); // 200ms para debounce
    bool LerBotao;
    
    while (1) {
        // Lê o estado atual do botão
        LerBotao = gpio_get(botaoA);
        
        // Verifica se o botão foi pressionado
        if (LerBotao == 0 && estadoBotao == 1) {
            // Verifica se o tempode de debounce foi atingido
            if ((xTaskGetTickCount() - BotaoDebounce) > tempoDebounce) {
                // Muda o modo
                ModoSemaforo = !ModoSemaforo;
                BotaoDebounce = xTaskGetTickCount();
            }
        }
        
        // Atualiza o estado do botão
        estadoBotao = LerBotao;
        
        // Delay baixo - se for alto trava com freqência
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//=====  Task responsável por atualizar Display  =====//

void vDisplay3Task()
{
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL);                                        // Pull up the clock line
    ssd1306_t ssd;                                                // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd);                                         // Configura o display
    ssd1306_send_data(&ssd);                                      // Envia os dados para o display
    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    char str_y[5]; // Buffer para armazenar a string
    int contador = 0;
    bool cor = true;
    while (true)
    {
        sprintf(str_y, "%d", contador); // Converte em string
        contador++;                     // Incrementa o contador
        ssd1306_fill(&ssd, !cor);                          // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);           // Desenha uma linha
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);           // Desenha uma linha
        ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 6); // Desenha uma string
        ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 16);  // Desenha uma string
        ssd1306_draw_string(&ssd, "  SEMAFORO", 10, 28);   // Desenha uma string
        ssd1306_draw_string(&ssd, "Semaforo", 10, 41);    // Desenha uma string
        if (ModoSemaforo == true) {
            ssd1306_draw_string(&ssd, " Diurno ", 10, 41);   // Desenha uma string
        }   else {
            ssd1306_draw_string(&ssd, " Noturno ", 10, 41);   // Desenha uma string
        }
        if (gpio_get(LED_G) && !gpio_get(LED_R)){
            ssd1306_draw_string(&ssd, " SIGA ", 10, 52);   // Desenha uma string
        } else if (gpio_get(LED_G) && gpio_get(LED_R)){
            ssd1306_draw_string(&ssd, " ATENCAO ", 10, 52);   // Desenha uma string
        } else if (!gpio_get(LED_G) && gpio_get(LED_R)){
            ssd1306_draw_string(&ssd, " PARE ", 10, 52);   // Desenha uma string
        }

        
        ssd1306_send_data(&ssd);                          // Atualiza o display
        sleep_ms(200);
    }
}

//=====  Bootsel  =====//

void gpio_irq_handler(uint gpio, uint32_t events)
{
    absolute_time_t now = get_absolute_time();
    int64_t diff = absolute_time_diff_us(last_interrupt_time, now);
    if (diff < 250000) return;
    last_interrupt_time = now;

    if (gpio == botaoB) {
        reset_usb_boot(0, 0);       // Bootsel                
    }
}


//=====  Configurações para a Matriz de LEDs  =====//

void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b);
void npClear();
void npInit(uint pin);
void npWrite();
void npDisplayDigit(int digit);
// Estrutura para representar um pixel RGB na matriz de LEDs
struct pixel_t {
    uint8_t G, R, B;           // Componentes verde, vermelho e azul
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;       // Tipo para LEDs NeoPixel

// Variáveis globais
npLED_t leds[LED_COUNT];       // Array para armazenar estado dos LEDs
PIO np_pio;                    // Instância do PIO para controle da matriz de LEDs

// Matrizes para exibição de dígitos na matriz de LEDs (5x5 pixels, RGB)
// Cada dígito/situação é representado por uma matriz de cores
const uint8_t digits[4][5][5][3] = {
    // VERDE = SIGA
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 100, 0}, {0, 100, 0}, {0, 100, 0}, {0, 0, 0}},
        {{0, 100, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 100, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // AMARELO = ATENÇÃO
    {
        {{0, 0, 0}, {0, 0, 0}, {100, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 100, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // VERMELHO = PARE
    {
        {{100, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {100, 0, 0}},
        {{0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}},
        {{100, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {100, 0, 0}}
    },
    // OFF = Desligado
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    }
};


// Define as cores de um LED na matriz
void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b) {
    leds[index].R = r; // Define componente vermelho
    leds[index].G = g; // Define componente verde
    leds[index].B = b; // Define componente azul
}

// Limpa a matriz de LEDs, exibindo o dígito 10 (padrão para limpar)
void npClear() {
    npDisplayDigit(4);
}

// Inicializa a matriz de LEDs WS2812B usando PIO
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program); // Carrega programa PIO
    np_pio = pio0; // Usa PIO0
    sm = pio_claim_unused_sm(np_pio, true); // Reserva máquina de estado
    // Inicializa programa PIO com pino e frequência
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);
    npClear(); // Limpa a matriz
}

// Escreve os dados dos LEDs na matriz
void npWrite() {
    for (uint i = 0; i < LED_COUNT; i++) {
        // Envia componentes G, R, B para o PIO em sequência
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100); // Pequeno atraso para estabilizar
}

// Calcula o índice de um LED na matriz com base em coordenadas (x, y)
int getIndex(int x, int y) {
    if (y % 2 == 0) {
        return 24 - (y * 5 + x); // Linhas pares: ordem direta
    } else {
        return 24 - (y * 5 + (4 - x)); // Linhas ímpares: ordem invertida
    }
}

// Exibe um dígito na matriz de LEDs
void npDisplayDigit(int digit) {
    for (int coluna = 0; coluna < 5; coluna++) {
        for (int linha = 0; linha < 5; linha++) {
            int posicao = getIndex(linha, coluna); // Calcula índice do LED
            // Define cores do LED com base na matriz de dígitos
            npSetLED(
                posicao,
                digits[digit][coluna][linha][0], // R
                digits[digit][coluna][linha][1], // G
                digits[digit][coluna][linha][2]  // B
            );
        }
    }
    npWrite(); // Escreve na matriz
}

//=====  Task responsável pela Matriz de LEDs  =====//

void vTaskMatriz(void *pvParameters) {
    while (1) {
        int local_estado = estado_semaforo; // Cópia local
        bool local_ModoSemaforo = ModoSemaforo; // Cópia local
        if (local_ModoSemaforo && Noite == false) {
            npDisplayDigit(local_estado);
        } else if (!local_ModoSemaforo && Noite == true) {
           if (gpio_get(LED_G) && gpio_get(LED_R)){
            npDisplayDigit(1);
           } else {
            npDisplayDigit(3);
           }
        }
        vTaskDelay(pdMS_TO_TICKS(150)); // Atraso para liberar CPU
    }
}

int main()
{
    stdio_init_all();
    npInit(LED_PIN);

    // Configurando Botões
    gpio_init(botaoA);
    gpio_init(botaoB);
    gpio_set_dir(botaoA, GPIO_IN);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoA);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoA, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    xTaskCreate(vTaskLED, "Task LED", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskBuzzer, "Task Buzzer", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vDisplay3Task, "Cont Task Disp3", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vAlternarModo, "Task Botao Flag", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskMatriz, "Task Matriz", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY -1, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}