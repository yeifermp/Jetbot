#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "driver/i2c.h"
#include "freertos/queue.h"

#define MOTOR_RESOLUTION 1000000                              // 10MHz, 1 tick = 0.1us
#define MOTOR_FREQ_HZ 2500                                     // 25KHz PWM
#define MOTOR_DUTY_TICK_MAX (MOTOR_RESOLUTION / MOTOR_FREQ_HZ)  // maximum value we can set for the duty cycle, in ticks
#define MOTOR_FRONT_LEFT_A 27
#define MOTOR_FRONT_LEFT_B 14
#define MOTOR_FRONT_RIGHT_A 25  
#define MOTOR_FRONT_RIGHT_B 26
#define MOTOR_BACK_LEFT_A 33
#define MOTOR_BACK_LEFT_B 32
#define MOTOR_BACK_RIGHT_A 19
#define MOTOR_BACK_RIGHT_B 18
#define DATA_LENGTH 1

static bdc_motor_handle_t front_left_motor = NULL;
static bdc_motor_handle_t front_right_motor = NULL;
static bdc_motor_handle_t back_left_motor = NULL;
static bdc_motor_handle_t back_right_motor = NULL;
static uint8_t receive_buffer;

static const char* TAG = "PWM Controller";

static void motors_configure(bdc_motor_handle_t* motor, uint8_t a, uint8_t b, uint8_t group) {
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = MOTOR_FREQ_HZ,
        .pwma_gpio_num = a,
        .pwmb_gpio_num = b,
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = group,
        .resolution_hz = MOTOR_RESOLUTION,
    };

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, motor));
    ESP_ERROR_CHECK(bdc_motor_enable(*motor));  
}

static void motors_set_speed(uint32_t speed) {
    bdc_motor_set_speed(front_left_motor, speed);
    bdc_motor_set_speed(front_right_motor, speed);
    bdc_motor_set_speed(back_left_motor, speed);
    bdc_motor_set_speed(back_right_motor, speed);
}

static void motors_increase_speed(void) {
    for (uint32_t speed = 1; speed <= MOTOR_DUTY_TICK_MAX; speed++) {
        motors_set_speed(speed);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void motors_forward(void) {
    bdc_motor_forward(front_left_motor);
    bdc_motor_forward(front_right_motor);
    bdc_motor_forward(back_left_motor);
    bdc_motor_forward(back_right_motor);
    motors_increase_speed();
}

static void motors_reverse(void) {
    bdc_motor_reverse(front_left_motor);
    bdc_motor_reverse(front_right_motor);
    bdc_motor_reverse(back_left_motor);
    bdc_motor_reverse(back_right_motor);
    motors_increase_speed();
}

static void motors_stop(void) {
    for (uint32_t speed = MOTOR_DUTY_TICK_MAX; speed >= 1; speed--) {
        motors_set_speed(speed);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    bdc_motor_brake(front_left_motor);
    bdc_motor_brake(front_right_motor);
    bdc_motor_brake(back_left_motor);
    bdc_motor_brake(back_right_motor);
}

static void motors_right(void) {
    bdc_motor_forward(front_left_motor);
    bdc_motor_reverse(front_right_motor);
    bdc_motor_forward(back_left_motor);
    bdc_motor_reverse(back_right_motor);
    motors_increase_speed();
}

static void motors_left(void) {
    bdc_motor_reverse(front_left_motor);
    bdc_motor_forward(front_right_motor);
    bdc_motor_reverse(back_left_motor);
    bdc_motor_forward(back_right_motor);
    motors_increase_speed();
}

static void jetson_init(void) {
    i2c_config_t i2c_config = {
        .scl_io_num = 17,
        .sda_io_num = 5,
        .mode = I2C_MODE_SLAVE,
        .slave.slave_addr  = 0x15,
        .slave.addr_10bit_en = 0,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
    };

    int i2c_slave_port = I2C_NUM_0;
    i2c_param_config(i2c_slave_port, &i2c_config);
    i2c_driver_install(i2c_slave_port, I2C_MODE_SLAVE, 512, 512, 0);

    receive_buffer = 0;

    while (true) {
        int len = i2c_slave_read_buffer(i2c_slave_port, &receive_buffer, DATA_LENGTH, 10000 / portTICK_PERIOD_MS);
        if (len == 0) {
            continue;
        }

        switch (receive_buffer)
        {
        case 0:
            motors_stop();
            ESP_LOGI(TAG, "Stop.");
            break;

        case 1:
            motors_forward();
            ESP_LOGI(TAG, "Going forward...");
            break;

        case 2:
            motors_reverse();
            ESP_LOGI(TAG, "Going backwards...");
            break;
        
        case 3:
            motors_left();
            ESP_LOGI(TAG, "Going left...");
            break;
        
        case 4:
            motors_right();
            ESP_LOGI(TAG, "Going right...");
            break;
        
        default:
            break;
        }
    }
}

void app_main(void) {    
    motors_configure(&front_left_motor, MOTOR_FRONT_LEFT_A, MOTOR_FRONT_LEFT_B, 0);
    motors_configure(&front_right_motor, MOTOR_FRONT_RIGHT_A, MOTOR_FRONT_RIGHT_B, 0);
    motors_configure(&back_left_motor, MOTOR_BACK_LEFT_A, MOTOR_BACK_LEFT_B, 0);
    motors_configure(&back_right_motor, MOTOR_BACK_RIGHT_A, MOTOR_BACK_RIGHT_B, 1);

    motors_set_speed(1);

    jetson_init();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}