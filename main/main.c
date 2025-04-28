#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "Fusion.h"

#include <stdio.h>
#include <math.h>

#define SAMPLE_PERIOD (0.01f) // 10ms -> 100Hz

typedef struct {
    uint8_t axis;    // 0: eixo X, 1: eixo Y, 2: Clique
    int16_t value;
} mouse_event_t;

QueueHandle_t xQueuePos;

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
    }

    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
    }

    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };
        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        mouse_event_t value_X;
        value_X.axis = 0;
        value_X.value = euler.angle.roll;

        mouse_event_t value_Y;
        value_Y.axis = 1;
        value_Y.value = euler.angle.pitch * 0.7f;

        mouse_event_t click_X;
        click_X.axis = 2;
        click_X.value = (accelerometer.axis.x)*105; 

        xQueueSend(xQueuePos, &value_X, portMAX_DELAY);
        xQueueSend(xQueuePos, &value_Y, portMAX_DELAY);

        if (fabsf(click_X.value) > 92){
            xQueueSend(xQueuePos, &click_X, portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *p) {
    mouse_event_t event;
    while (1) {
        if (xQueueReceive(xQueuePos, &event, portMAX_DELAY)) {
            int16_t valor = event.value;

            uint8_t axis = event.axis;
            uint8_t val_0 = (uint8_t)(valor & 0xFF);
            uint8_t val_1 = (uint8_t)((valor >> 8) & 0xFF);
            uint8_t eop = 0xFF;

            putchar_raw(eop);
            putchar_raw(axis);
            putchar_raw(val_0);
            putchar_raw(val_1);
        }
    }
}

int main() {

    stdio_init_all();

    xQueuePos = xQueueCreate(32, sizeof(mouse_event_t));
    if (xQueuePos == NULL) {
        printf("Erro ao criar fila\n");
        while (1);
    }

    xTaskCreate(mpu6050_task, "MPU6050 Task", 4096, NULL, 1, NULL);
    xTaskCreate(uart_task, "UART Task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1);
}
