#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"

#include "lwip/sockets.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "cJSON.h"

#define FLUENTD_IP            "192.168.2.20"   // IP address of Fluentd
#define FLUENTD_PORT           8888            // Port of FLuentd
#define FLUENTD_TAG            "/sensor"       // Fluentd tag
#define WIFI_HOSTNAME          "ESP32-geiger"  // module's hostname

#include "wifi_config.h"
// wifi_config.h should define followings.
// #define WIFI_SSID "XXXXXXXX"            // WiFi SSID
// #define WIFI_PASS "XXXXXXXX"            // WiFi Password

#define GEIGER_UART_NUM        UART_NUM_2
#define GEIGER_UART_TXD        GPIO_NUM_25
#define GEIGER_UART_RXD        GPIO_NUM_33
#define GEIGER_UART_BUF_SIZE   256
#define GEIGER_AVE_NUM         6
#define LND712_USVH_FACTOR     0.00812

#define WIFI_CONNECT_TIMEOUT   20
#define TAG                    "geiger"

#define EXPECTED_RESPONSE "HTTP/1.1 200 OK"
#define REQUEST "POST http://" FLUENTD_IP FLUENTD_TAG " HTTP/1.0\r\n" \
    "Content-Type: application/x-www-form-urlencoded\r\n" \
    "Content-Length: %d\r\n" \
    "\r\n" \
    "json=%s"

#define ARRAY_SIZEOF(array) (sizeof(array)/sizeof(array[0]))

SemaphoreHandle_t sense_done = NULL;
SemaphoreHandle_t wifi_conn_done = NULL;

volatile uint16_t sense_data[GEIGER_AVE_NUM];
volatile uint8_t sense_num = 0;

//////////////////////////////////////////////////////////////////////
// Fluentd Function
float get_usvh()
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < ARRAY_SIZEOF(sense_data); i++) {
        sum += sense_data[i];
    }
    return ((float)sum) / ARRAY_SIZEOF(sense_data) * LND712_USVH_FACTOR;
}

static cJSON *sense_json(float usvh, wifi_ap_record_t *ap_record)
{
    cJSON *root = cJSON_CreateArray();
    cJSON *item = cJSON_CreateObject();

    cJSON_AddNumberToObject(item, "uSv/h", usvh);

    cJSON_AddStringToObject(item, "hostname", WIFI_HOSTNAME);
    cJSON_AddNumberToObject(item, "wifi_ch", ap_record->primary);
    cJSON_AddNumberToObject(item, "wifi_rssi", ap_record->rssi);

    cJSON_AddNumberToObject(item, "self_time", 0); // for Fluentd

    cJSON_AddItemToArray(root, item);

    return root;
}

static esp_err_t connect_server(int *sock)
{
    struct sockaddr_in server;

    *sock = socket(AF_INET, SOCK_STREAM, 0);

    server.sin_family = AF_INET;
    server.sin_port = htons(FLUENTD_PORT);
    server.sin_addr.s_addr = inet_addr(FLUENTD_IP);

    if (connect(*sock, (struct sockaddr *)&server, sizeof(server)) != 0) {
        ESP_LOGE(TAG, "FLUENTD CONNECT FAILED errno=%d", errno);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "FLUENTD CONNECT SUCCESS");

    return ESP_OK;
}

static esp_err_t process_sense_data(float usvh)
{
    wifi_ap_record_t ap_record;
    char buffer[sizeof(EXPECTED_RESPONSE)];
    esp_err_t ret = ESP_FAIL;

    ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_record));

    int sock = -1;
    ESP_ERROR_CHECK(connect_server(&sock));

    cJSON *json = sense_json(usvh, &ap_record);
    char *json_str = cJSON_PrintUnformatted(json);

    do {
        if (dprintf(sock, REQUEST, strlen("json=") + strlen(json_str), json_str) < 0) {
            ESP_LOGE(TAG, "FLUENTD POST FAILED");
            break;
        }

        bzero(buffer, sizeof(buffer));
        read(sock, buffer, sizeof(buffer)-1);

        if (strcmp(buffer, EXPECTED_RESPONSE) != 0) {
            ESP_LOGE(TAG, "FLUENTD POST FAILED");
            break;
        }
        ret = ESP_OK;
        ESP_LOGI(TAG, "FLUENTD POST SUCCESSFUL");
    } while (0);

    close(sock);
    cJSON_Delete(json);

    return ret;
}

//////////////////////////////////////////////////////////////////////
// Wifi Function
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, WIFI_HOSTNAME));
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        xSemaphoreGive(wifi_conn_done);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
        xSemaphoreGive(wifi_conn_done);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void init_wifi()
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

#ifdef WIFI_SSID
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    wifi_config_t wifi_config_cur;
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &wifi_config_cur));

    if (strcmp((const char *)wifi_config_cur.sta.ssid, (const char *)wifi_config.sta.ssid) ||
        strcmp((const char *)wifi_config_cur.sta.password, (const char *)wifi_config.sta.password)) {
        ESP_LOGI(TAG, "SAVE WIFI CONFIG");
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    }
#endif
}

static esp_err_t wifi_connect()
{
    xSemaphoreTake(wifi_conn_done, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_wifi_start());
    if (xSemaphoreTake(wifi_conn_done, 10000 / portTICK_RATE_MS) == pdTRUE) {
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "WIFI CONNECT TIMECOUT");
        return ESP_FAIL;
    }
}

static void wifi_disconnect()
{
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_stop());
}

//////////////////////////////////////////////////////////////////////
// Wifi Task
static void fluentd_post_task()
{
    float usvh;

    while (1) {
        xSemaphoreTake(sense_done, portMAX_DELAY);
        usvh = get_usvh();
        sense_num = 0;

        ESP_LOGI(TAG, "uSv/h = %.3f", usvh);

        ESP_ERROR_CHECK(wifi_connect());
        process_sense_data(usvh);

        wifi_disconnect();
    }
}

//////////////////////////////////////////////////////////////////////
// UART Task
static void uart_read_task()
{
    uint8_t buf[GEIGER_UART_BUF_SIZE];
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(GEIGER_UART_NUM, &uart_config);
    uart_set_pin(GEIGER_UART_NUM, GEIGER_UART_TXD, GEIGER_UART_RXD,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GEIGER_UART_NUM, GEIGER_UART_BUF_SIZE*2, 0, 0, NULL, 0);

    while (1) {
        memset(buf, 0, GEIGER_UART_BUF_SIZE);
        int len = uart_read_bytes(GEIGER_UART_NUM, buf, GEIGER_UART_BUF_SIZE,
                                  10/portTICK_RATE_MS);

        if (len > 0) {
            if (sense_num == ARRAY_SIZEOF(sense_data)) {
                continue;
            }
            sense_data[sense_num++] = (uint16_t)atoi((char *)buf);
            if (sense_num == ARRAY_SIZEOF(sense_data)) {
                xSemaphoreGive(sense_done);
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////
void app_main()
{
    ESP_LOGI(TAG, "APP START");

    vSemaphoreCreateBinary(sense_done);
    vSemaphoreCreateBinary(wifi_conn_done);

    xSemaphoreTake(sense_done, portMAX_DELAY);

    init_wifi();

    xTaskCreate(uart_read_task, "uart_read", 1024, NULL, 10, NULL);
    xTaskCreate(fluentd_post_task, "fluentd_post", 102400, NULL, 10, NULL);
}
