#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include "AlertaEnchente.pio.h"
#include "pico/bootrom.h"

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define ADC_JOYSTICK_X 26
#define ADC_JOYSTICK_Y 27
#define LED_red 13
#define LED_MATRIX_PIN 7
#define BUZZER_PIN 21
#define botaoB 6

// TIPOS E VARIÁVEIS GLOBAIS
typedef struct { PIO pio; uint sm; } PioConfig;

typedef struct {
    uint16_t x_pos;
    uint16_t y_pos;
    bool alerta;
} joystick_data_t;

QueueHandle_t xQueueDisplay;
QueueHandle_t xQueueLedred;
QueueHandle_t xQueueMatrix;
QueueHandle_t xQueueBuzzer;

// DECLARAÇÕES DE FUNÇÕES
void vJoystickTask(void *params);
void vDisplayTask(void *params);
void vLedredTask(void *params);
void vMatrixTask(void *params);
void vBuzzerTask(void *params);
void gpio_irq_handler(uint gpio, uint32_t events);

// FUNÇÃO PRINCIPAL
int main() {
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();

    xQueueDisplay = xQueueCreate(5, sizeof(joystick_data_t));
    xQueueLedred = xQueueCreate(5, sizeof(joystick_data_t));
    xQueueMatrix = xQueueCreate(5, sizeof(joystick_data_t));
    xQueueBuzzer = xQueueCreate(5, sizeof(joystick_data_t));

    xTaskCreate(vJoystickTask, "Joystick Task", 256, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 512, NULL, 1, NULL);
    xTaskCreate(vLedredTask, "LED red Task", 256, NULL, 1 , NULL);
    xTaskCreate(vMatrixTask, "Matrix Task", 256, NULL, 1, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer", 256, NULL, 1, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}

// Leitura do joystick e envio para as filas
void vJoystickTask(void *params) {
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);
    adc_init();

    joystick_data_t joydata;

    while (true) {
        adc_select_input(0); // Y
        joydata.y_pos = adc_read();
        adc_select_input(1); // X
        joydata.x_pos = adc_read();

        uint8_t agua = (joydata.x_pos * 100) / 4095;
        uint8_t chuva = (joydata.y_pos * 100) / 4095;
        joydata.alerta = (agua >= 70 || chuva >= 80);

        xQueueSend(xQueueDisplay, &joydata, 0);
        xQueueSend(xQueueLedred, &joydata, 0);
        xQueueSend(xQueueMatrix, &joydata, 0);
        xQueueSend(xQueueBuzzer, &joydata, 0);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Atualiza display OLED com dados do joystick
void vDisplayTask(void *params) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_t ssd;
    ssd1306_init(&ssd, 128, 64, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    joystick_data_t joydata;
    char str_agua[16];
    char str_chuva[16];

    while (true) {
        if (xQueueReceive(xQueueDisplay, &joydata, portMAX_DELAY) == pdTRUE) {
            uint8_t agua = (joydata.x_pos * 100) / 4095;
            uint8_t chuva = (joydata.y_pos * 100) / 4095;

            ssd1306_fill(&ssd, 0);

            if (joydata.alerta) {
                ssd1306_draw_string(&ssd, "! ALERTA !", 20, 0);
            }

            snprintf(str_agua, sizeof(str_agua), "AGUA:  %3d%%", agua);
            snprintf(str_chuva, sizeof(str_chuva), "CHUVA: %3d%%", chuva);

            ssd1306_draw_string(&ssd, str_agua, 10, 20);
            ssd1306_draw_string(&ssd, str_chuva, 10, 40);
            ssd1306_send_data(&ssd);
        }
    }
}

// Controle de LED vermelho via PWM
void vLedredTask(void *params) {
    gpio_init(LED_red);
    gpio_set_dir(LED_red, GPIO_OUT);

    joystick_data_t joydata;

    while (true) {
        if (xQueueReceive(xQueueLedred, &joydata, portMAX_DELAY) == pdTRUE) {
            gpio_put(LED_red, joydata.alerta);
        }
    }
}

// Controle da matriz de LEDs com PIO
void vMatrixTask(void *params) {
    PioConfig led_cfg;
    led_cfg.pio = pio0;
    led_cfg.sm = pio_claim_unused_sm(led_cfg.pio, true);
    uint offset = pio_add_program(led_cfg.pio, &pio_matrix_program);
    pio_matrix_program_init(led_cfg.pio, led_cfg.sm, offset, LED_MATRIX_PIN);

    joystick_data_t joydata;

    while (true) {
        if (xQueueReceive(xQueueMatrix, &joydata, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (int r = 0; r < 5; r++) {
                for (int c = 0; c < 5; c++) {
                    uint32_t color = joydata.alerta ?
                        ((c == 2 && (r == 4 || r == 3 || r == 2 || r == 0)) ? 0x00FF0000 : 0) :
                        0x00000000;

                    pio_sm_put_blocking(led_cfg.pio, led_cfg.sm, color);
                }
            }
        }
    }
}

// Controle de som no buzzer via PWM
void vBuzzerTask(void *params) {
    uint buzzer_slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 4.0f);
    pwm_config_set_wrap(&cfg, 15625);
    pwm_init(buzzer_slice, &cfg, true);

    joystick_data_t joydata;

    while (true) {
        if (xQueueReceive(xQueueLedred, &joydata, portMAX_DELAY) == pdTRUE) {
            if (joydata.alerta) {
                pwm_set_gpio_level(BUZZER_PIN, 7812); // 1KHz beep
                vTaskDelay(pdMS_TO_TICKS(500));
                pwm_set_gpio_level(BUZZER_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
            } else {
                pwm_set_gpio_level(BUZZER_PIN, 0);
            }
        }
    }
}

// Interrupção no botão B para reset USB
void gpio_irq_handler(uint gpio, uint32_t events) {
    reset_usb_boot(0, 0);
}
