#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_SSID       "<ssid>"
#define WIFI_PASS       "<pass>"
#define WIFI_MAX_RETRY  5

#define MQTT_URL        "mqtt://<domain>"
#define MQTT_TOPIC      "<topic>"

#define DETECT_DISTANCE_MM 500

#define UART_NUM_USED      UART_NUM_1
#define UART_TX_GPIO_NUM   15
#define UART_RX_GPIO_NUM   14
#define BUF_SIZE           1024

static const char *TAG = "uart_bridge";

// Global MQTT client handle.
static esp_mqtt_client_handle_t mqtt_client = NULL;

// Global retry counter for Wi‑Fi connection.
static int s_retry_num = 0;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/**
 * @brief Wi‑Fi event handler.
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch(event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                ESP_LOGI(TAG, "Wi-Fi started, attempting connection...");
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                if (s_retry_num < WIFI_MAX_RETRY) {
                    esp_wifi_connect();
                    s_retry_num++;
                    ESP_LOGI(TAG, "Retrying to connect to the Wi-Fi network");
                } else {
                    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                }
                ESP_LOGI(TAG, "Failed to connect to the Wi-Fi network");
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Initialize Wi‑Fi as a station.
 */
static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    // Initialize the TCP/IP stack.
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop. This is required before creating default Wi‑Fi interfaces.
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default Wi‑Fi station.
    esp_netif_create_default_wifi_sta();

    // Initialize Wi‑Fi with default configuration.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers for Wi‑Fi and IP events.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            // Note: Providing a password sets WPA/WPA2 security. For open networks, leave empty.
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    // Wait for connection.
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                             WIFI_CONNECTED_BIT,
                                             pdFALSE,
                                             pdFALSE,
                                             portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGW(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    }
}

/**
 * @brief Publishes on MQTT topic "europalaan/turing" with payload "used" or "free"
 *        depending on the detected argument. Only publishes if the detection
 *        state changes.
 */
static void update_detection(bool detected) {
    const char *payload = detected ? "used" : "free";
    int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);
    ESP_LOGI(TAG, "MQTT publish: topic %s, payload %s, msg_id=%d", MQTT_TOPIC, payload, msg_id);
}

/**
 * @brief Task that reads data from the external UART, decodes packets, and updates detection.
 */
static void uart_bridge_task(void *arg)
{
    uint8_t buf[BUF_SIZE];
    static uint8_t packet[30]; // Full packet: 4(header) + 24(targets) + 2(footer) = 30 bytes
    static int packet_pos = 0;
    const uint8_t header[] = {0xAA, 0xFF, 0x03, 0x00};
    const uint8_t footer[] = {0x55, 0xCC};

    while (1) {
        int len = uart_read_bytes(UART_NUM_USED, buf, sizeof(buf), 20 / portTICK_PERIOD_MS);
        for (int i = 0; i < len; i++) {
            uint8_t byte = buf[i];

            // Buffer packet data.
            if (packet_pos < 4) {
                if (byte == header[packet_pos]) {
                    packet[packet_pos++] = byte;
                } else {
                    packet_pos = 0; // Reset if header sequence fails.
                }
            } else {
                packet[packet_pos++] = byte;
                if (packet_pos == 30) {
                    // Check for a valid footer.
                    if (packet[28] == footer[0] && packet[29] == footer[1]) {
                        // Valid packet. Decode target 1 (bytes 4..11).
                        int targetId;
                        bool detected = false;
                        for (targetId = 0; targetId < 3; targetId++) {
                            uint8_t *target = &packet[4 + targetId*8];

                            // Decode X coordinate.
                            uint16_t x_raw = target[0] + (target[1] << 8);
                            int x;
                            if (x_raw > 32767) {
                                x = x_raw - 32768;
                            } else {
                                x = -x_raw;
                            }

                            // Decode Y coordinate.
                            uint16_t y_raw = target[2] + (target[3] << 8);
                            int y;
                            if (y_raw > 32767) {
                                y = y_raw - 32768;
                            } else {
                                y = -y_raw;
                            }

                            // Decode speed.
                            uint16_t speed_raw = target[4] + (target[5] << 8);
                            int speed;
                            if (speed_raw > 32767) {
                                speed = speed_raw - 32768;
                            } else {
                                speed = -speed_raw;
                            }

                            // Decode distance resolution.
                            int16_t resolution = target[6] + (target[7] << 8);

                            ESP_LOGI(TAG, "Target %d:\n  X = %d mm\n  Y = %d mm\n  Speed = %d cm/s\n  Resolution = %d mm", 
                                    targetId+1, x, y, speed, resolution);

                            // Detection logic: if y-coordinate is below 500 mm, a valid detection.
                            if (y < DETECT_DISTANCE_MM && resolution != 0) {
                                detected = true;
                                break;
                            } 
                        }

                        update_detection(detected);
                        break;
                    } else {
                        ESP_LOGW(TAG, "Invalid footer, discarding packet.");
                    }
                    packet_pos = 0; // Reset packet buffer.
                }
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief MQTT event handler with corrected signature.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        default:
            break;
    }
}

/**
 * @brief Initialize and start the MQTT client.
 */
static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URL,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

void app_main(void)
{
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi‑Fi.
    wifi_init_sta();

    // Configure UART parameters for the external device.
    uart_config_t uart_config = {
        .baud_rate = 256000,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // UART0 is used for the USB console (logging); install a minimal driver on UART0.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

    // Install the driver for UART1.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_USED, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_USED, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_USED, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART for external serial device initialized on TX=%d, RX=%d", UART_TX_GPIO_NUM, UART_RX_GPIO_NUM);

    // Start the MQTT client.
    mqtt_app_start();

    // Create the UART processing task.
    xTaskCreate(uart_bridge_task, "uart_bridge_task", 4096, NULL, 5, NULL);
}
