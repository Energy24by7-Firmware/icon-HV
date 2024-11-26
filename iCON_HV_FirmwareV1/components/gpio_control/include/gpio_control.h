// gpio_control.h
#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include "driver/gpio.h"

// GPIO pin definitions
#define LED_GREEN_PIN 12
#define LED_BLUE_PIN 14
#define LED_RED_PIN 27
#define BUZZER_PIN 10
#define FAN1_PIN 9
#define FAN2_PIN 4

// Declare current_led as extern
extern gpio_num_t current_led;

// Function prototypes
void gpio_control_init(void);

void led_on(gpio_num_t led);
void led_off(gpio_num_t led);
void blink_led(gpio_num_t led, uint32_t interval_ms);

void led_magenta_on(void);
void led_magenta_off(void);
void blink_magenta_led(uint32_t interval_ms);

void buzzer_on(void);
void buzzer_off(void);

void fan1_on(void);
void fan1_off(void);
void fan2_on(void);
void fan2_off(void);

#endif // GPIO_CONTROL_H
