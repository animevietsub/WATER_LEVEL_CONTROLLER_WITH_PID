#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "PID.h"

static const char *TAG_UART = "UART_WATER_LEVEL";
static const char *TAG_PID = "PID_WATER_LEVEL";
static const char *TAG_NVS = "NVS_WATER_LEVEL";

const uart_port_t uart_num = UART_NUM_2;
const char SONIC_MEASURE_REQUEST[] = {0xFF};

#define SONIC_TX_PIN 5
#define SONIC_RX_PIN 18
#define UART_BUFFER_SIZE (1024) // Size for reveiving distance DATA
#define SONIC_BUFFER_SIZE (64)  // Size from SONIC

PIDdata pid_value = {
    .PIDinput_prev = 0,
    .PIDsetpoint = 300,
    .PIDkP = 0,
    .PIDkI = 0,
    .PIDkD = 0,
    .PIDPerr = 0,
    .PIDIerr = 0,
    .PIDDerr = 0,
    .PIDPerrmin = 0,
    .PIDPerrmax = 0,
    .PIDIerrmin = 0,
    .PIDIerrmax = 0,
};

static void PID_process(uint16_t input)
{
    pid_value.PIDPerr = pid_value.PIDsetpoint - input;
    pid_value.PIDIerr += pid_value.PIDPerr;
    pid_value.PIDDerr = pid_value.PIDinput_prev - input;
    pid_value.PIDinput_prev = input;
    ESP_LOGI(TAG_PID, "PIDPerr = %f", pid_value.PIDPerr);
    ESP_LOGI(TAG_PID, "PIDIerr = %f", pid_value.PIDIerr);
    ESP_LOGI(TAG_PID, "PIDDerr = %f", pid_value.PIDDerr);
}

void UART_Init()
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_set_pin(uart_num, SONIC_RX_PIN, SONIC_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, 0, 0));
}

static void UART_getDistance()
{
    uint8_t *data = (uint8_t *)malloc(SONIC_BUFFER_SIZE);
    while (1)
    {
        uart_write_bytes(uart_num, SONIC_MEASURE_REQUEST, sizeof(SONIC_MEASURE_REQUEST) / sizeof(char));
        int len = uart_read_bytes(uart_num, data, (SONIC_BUFFER_SIZE - 1), pdMS_TO_TICKS(50));
        if (len)
        {
            if (data[0] == 0xFF)
            {
                ESP_LOGI(TAG_UART, "Recv data: %d, %d, %d", data[0], data[1], data[2]);
                ESP_LOGI(TAG_UART, "Distance: %d mm", (uint16_t)((data[1] << 8) + data[2]));
                PID_process((uint16_t)((data[1] << 8) + data[2]));
            }
            else
            {
                ESP_LOGE(TAG_UART, "Error in getting distance");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void NVS_Init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        ESP_LOGI(TAG_NVS, "Flash Erase!");
    }
    ESP_ERROR_CHECK(err);
}

esp_err_t write_struct(const char *key, void *write_struct, size_t size)
{
    nvs_handle_t handle;
    esp_err_t err;
    err = nvs_open("storage", NVS_READWRITE, &handle);
    err = nvs_set_blob(handle, key, write_struct, size);
    err = nvs_commit(handle);

    nvs_close(handle);
    return err;
}

esp_err_t read_struct(const char *key, void *read_struct, size_t size)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &handle);
    err = nvs_get_blob(handle, key, read_struct, &size);

    nvs_close(handle);
    return err;
}

void NVS_readPID()
{
    ESP_LOGI(TAG_NVS, "Getting PID valued from NVS");
    esp_err_t err = read_struct("pid_value", &pid_value, sizeof(pid_value));
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG_NVS, "Geting PID values successfully");
        ESP_LOGI(TAG_NVS, "PIDinput_prev = %d", pid_value.PIDinput_prev);
        ESP_LOGI(TAG_NVS, "PIDsetpoint = %d", pid_value.PIDsetpoint);
        ESP_LOGI(TAG_NVS, "PIDkP = %f", pid_value.PIDkP);
        ESP_LOGI(TAG_NVS, "PIDkI = %f", pid_value.PIDkI);
        ESP_LOGI(TAG_NVS, "PIDkD = %f", pid_value.PIDkD);

        ESP_LOGI(TAG_NVS, "PIDPerr = %f", pid_value.PIDPerr);
        ESP_LOGI(TAG_NVS, "PIDIerr = %f", pid_value.PIDIerr);
        ESP_LOGI(TAG_NVS, "PIDDerr = %f", pid_value.PIDDerr);

        ESP_LOGI(TAG_NVS, "PIDPerrmin = %f", pid_value.PIDPerrmin);
        ESP_LOGI(TAG_NVS, "PIDPerrmax = %f", pid_value.PIDPerrmax);
        ESP_LOGI(TAG_NVS, "PIDIerrmin = %f", pid_value.PIDIerrmin);
        ESP_LOGI(TAG_NVS, "PIDIerrmax = %f", pid_value.PIDIerrmax);
    }
    else
    {
        ESP_LOGE(TAG_NVS, "Error (%s) in reading\n", esp_err_to_name(err));
    }
}

void NVS_writePID()
{
    ESP_LOGI(TAG_NVS, "Writing PID values to NVS");
    esp_err_t err = write_struct("pid_value", &pid_value, sizeof(pid_value));
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG_NVS, "Geting PID values successfully");
        ESP_LOGI(TAG_NVS, "PIDinput_prev = %d", pid_value.PIDinput_prev);
        ESP_LOGI(TAG_NVS, "PIDsetpoint = %d", pid_value.PIDsetpoint);
        ESP_LOGI(TAG_NVS, "PIDkP = %f", pid_value.PIDkP);
        ESP_LOGI(TAG_NVS, "PIDkI = %f", pid_value.PIDkI);
        ESP_LOGI(TAG_NVS, "PIDkD = %f", pid_value.PIDkD);

        ESP_LOGI(TAG_NVS, "PIDPerr = %f", pid_value.PIDPerr);
        ESP_LOGI(TAG_NVS, "PIDIerr = %f", pid_value.PIDIerr);
        ESP_LOGI(TAG_NVS, "PIDDerr = %f", pid_value.PIDDerr);

        ESP_LOGI(TAG_NVS, "PIDPerrmin = %f", pid_value.PIDPerrmin);
        ESP_LOGI(TAG_NVS, "PIDPerrmax = %f", pid_value.PIDPerrmax);
        ESP_LOGI(TAG_NVS, "PIDIerrmin = %f", pid_value.PIDIerrmin);
        ESP_LOGI(TAG_NVS, "PIDIerrmax = %f", pid_value.PIDIerrmax);
    }
    else
    {
        ESP_LOGE(TAG_NVS, "Error (%s) in writing\n", esp_err_to_name(err));
    }
}

void app_main(void)
{
    UART_Init();
    NVS_Init();
    NVS_writePID();
    NVS_readPID();
    xTaskCreate(UART_getDistance, "UART_GET_DISTANCE", 1024, NULL, 10, NULL);
}
