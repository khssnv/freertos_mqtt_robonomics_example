#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"

#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const char *MQTT_TOPIC_BASE = "/freertos_mqtt_robonomics_example/PMS3003";
char mcu_id[18] = {0};
char MQTT_TOPIC[50] = {0};
static const char *TAG = "FREERTOS_MQTT_ROBONOMICS";
static int mqtt_ready = 0;
static const int RX_BUF_SIZE = 1024;
#define TX_BUF_SIZE_BYTES 1024
#define TXD_PIN (GPIO_NUM_16)
#define RXD_PIN (GPIO_NUM_17)
typedef struct {
	uint16_t pm1_0;
	uint16_t pm2_5;
	uint16_t pm10;
	uint8_t sensor_idx;
} pm_data_t;
static uint8_t ucStorageBuf[TX_BUF_SIZE_BYTES];
StaticMessageBuffer_t txMsgStruct;
MessageBufferHandle_t txMsgBuf;
esp_mqtt_client_handle_t client;

static pm_data_t decode_pms3003_data(uint8_t* data, uint8_t start_byte) {
	pm_data_t pm = {
		.pm1_0 = ((data[start_byte]<<8) + data[start_byte+1]),
		.pm2_5 = ((data[start_byte+2]<<8) + data[start_byte+3]),
		.pm10 = ((data[start_byte+4]<<8) + data[start_byte+5])
    };
	return pm;
}

static void log_error_if_nonzero(const char * message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    client = event->client;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            mqtt_ready = 1;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            mqtt_ready = 0;
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI("SNTP", "Time synchronized");
}

static void time_init(void) {
    ESP_LOGI("INIT", "Time synchronization");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();

    time_t now = 0;
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    ESP_LOGI("INIT", "SNTP init done at %ld", now);
}

static void sensor_init(void) {
    ESP_LOGI("INIT", "Preparing UART");
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void uart_rx_task(void *arg)
{
    static const char *UART_RX_TASK_TAG = "UART_RX_TASK";
    esp_log_level_set(UART_RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
	time_t now;
    char txMsgStr[255];
    memset(txMsgStr, 0, 255);
    while (1) {
        ESP_LOGI(UART_RX_TASK_TAG, "Waiting for data from sensor");
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
		time(&now);
        ESP_LOGI(UART_RX_TASK_TAG, "Received %d bytes from sensor by UART", rxBytes);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(UART_RX_TASK_TAG, "Raw data (%d bytes): '%s'", rxBytes, (char*)data);
            ESP_LOG_BUFFER_HEXDUMP(UART_RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
			pm_data_t pm = decode_pms3003_data(data, 10); //atmospheric from 10th byte, standard from 4th
            sprintf(txMsgStr, "ts=%ld, PM1=%d, PM2.5=%d, PM10=%d", now, pm.pm1_0, pm.pm2_5, pm.pm10);
            ESP_LOGI(UART_RX_TASK_TAG, "Writing to buffer: '%s'", txMsgStr);
            xMessageBufferSend(txMsgBuf, (void*) txMsgStr, strlen(txMsgStr), pdMS_TO_TICKS(500));
            ESP_LOGI(UART_RX_TASK_TAG, "Message written to buffer");
        }
    }
    free(data);
}

static void mqtt_tx_task(void *arg)
{

    static const char *MQTT_TX_TASK_TAG = "MQTT_TX_TASK";
    esp_log_level_set(MQTT_TX_TASK_TAG, ESP_LOG_INFO);
    time_t now = 0;
	char txMsgStr[2048];
    memset(txMsgStr, 0, 255);
	int msg_id;
	while (1) {
		ESP_LOGI(MQTT_TX_TASK_TAG, "Waiting for message from buffer");
		xMessageBufferReceive(txMsgBuf, (void*)txMsgStr, sizeof(txMsgStr), portMAX_DELAY);
		ESP_LOGI(MQTT_TX_TASK_TAG, "Message received: %s", txMsgStr);
		if(!mqtt_ready) {
			ESP_LOGI(MQTT_TX_TASK_TAG, "MQTT is not ready, skipping");
			vTaskDelay(pdMS_TO_TICKS(500));
			continue;
		}
		ESP_LOGI(MQTT_TX_TASK_TAG, "Publishing");
		msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, txMsgStr, 0, 1, 0);
        time(&now);
		ESP_LOGI(MQTT_TX_TASK_TAG, "Message #%d published at %ld", msg_id, now);
	}
}

void app_main(void)
{
	// Init logs
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    uint8_t mcu_id_raw[6];
    esp_efuse_mac_get_default(mcu_id_raw);
    sprintf(mcu_id, "%02X:%02X:%02X:%02X:%02X:%02X", mcu_id_raw[0], mcu_id_raw[1], mcu_id_raw[2], mcu_id_raw[3], mcu_id_raw[4], mcu_id_raw[5]);
    ESP_LOGI(TAG, "[APP] MCU ID: %s", mcu_id);
    strcpy(MQTT_TOPIC, MQTT_TOPIC_BASE);
    strcat(MQTT_TOPIC, "-");
    strcat(MQTT_TOPIC, mcu_id);
    ESP_LOGI(TAG, "[APP] MQTT topic: %s", MQTT_TOPIC);

    // Configure logging by serial port
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    // Init
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect()); // connect by Wi-Fi
    time_init();
    sensor_init();

    txMsgBuf = xMessageBufferCreateStatic(sizeof(ucStorageBuf), ucStorageBuf, &txMsgStruct);
    xTaskCreate(uart_rx_task, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(mqtt_tx_task, "mqtt_tx_task", 1024*4, NULL, configMAX_PRIORITIES-1, NULL);

    mqtt_app_start();
}
