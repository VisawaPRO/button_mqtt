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

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/gpio.h"  // For GPIO control

// Define constants
#define BUTTON_GPIO 23
#define LED_GPIO 19       // Define GPIO for the LED
#define ESP_INTR_FLAG_DEFAULT 0  // Manually define ESP_INTR_FLAG_DEFAULT

static const char *TAG = "mqtt_example";

esp_mqtt_client_handle_t client;  // Global MQTT client handle

// Function prototypes
static void button_init(void);
static void led_init(void);
static void IRAM_ATTR button_isr_handler(void *arg);
static void button_task(void *arg);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void mqtt_app_start(void);
static void led_control(const char *message, int length);

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Control LED based on received MQTT message
 */
static void led_control(const char *message, int length)
{
    char trimmed_message[32];  // Adjust size as necessary
    int message_length = length < sizeof(trimmed_message) ? length : sizeof(trimmed_message) - 1;

    // Copy and null-terminate the message
    strncpy(trimmed_message, message, message_length);
    trimmed_message[message_length] = '\0';

    ESP_LOGI(TAG, "LED control message: %s (length: %d)", trimmed_message, strlen(trimmed_message));
    if (strcmp(trimmed_message, "ON") == 0) {
        gpio_set_level(LED_GPIO, 1); // Turn LED on
        ESP_LOGI(TAG, "LED is ON");
    } else if (strcmp(trimmed_message, "OFF") == 0) {
        gpio_set_level(LED_GPIO, 0); // Turn LED off
        ESP_LOGI(TAG, "LED is OFF");
    } else {
        ESP_LOGW(TAG, "Received unexpected message for LED control: %s", trimmed_message);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, "KMITL/SIET/65030229/LED", 1); // Subscribe to the LED control topic
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        // Check if the received topic is for the LED control
        if (strncmp(event->topic, "KMITL/SIET/65030229/LED", event->topic_len) == 0) {
            // Control LED based on the received message
            led_control((const char *)event->data, event->data_len);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

/*
 * @brief Initializes GPIO for the button and configures interrupt handling
 */
static void button_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;  // Interrupt on both edges (button press and release)
    io_conf.mode = GPIO_MODE_INPUT;         // Set as input mode
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO); // Bitmask for GPIO 23
    io_conf.pull_down_en = 0;               // No pull-down
    io_conf.pull_up_en = 1;                 // Enable pull-up resistor
    gpio_config(&io_conf);

    // Install GPIO ISR handler
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL); // Attach interrupt handler
}

/*
 * @brief Initializes GPIO for the LED
 */
static void led_init(void)
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT); // Set LED GPIO as output
}

/*
 * @brief ISR handler for button press
 */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    static uint32_t last_isr_time = 0;
    uint32_t current_time = xTaskGetTickCountFromISR();
    
    // Debounce logic
    if (current_time - last_isr_time > pdMS_TO_TICKS(50)) {
        if (gpio_get_level(BUTTON_GPIO) == 0) { // Button pressed
            xTaskCreate(button_task, "button_task", 2048, (void *)"1", 10, NULL);
        } else { // Button released
            xTaskCreate(button_task, "button_task", 2048, (void *)"0", 10, NULL);
        }
    }
    
    last_isr_time = current_time;
}

/*
 * @brief Task to publish message when button is pressed or released
 */
static void button_task(void *arg)
{
    const char *message = (const char *)arg;
    int msg_id = esp_mqtt_client_publish(client, "KMITL/SIET/65030229/Button", message, 0, 1, 0);
    ESP_LOGI(TAG, "Button state: %s, published message, msg_id=%d", message, msg_id);
    
    vTaskDelete(NULL); // Delete the task after execution
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    // Initialize button and LED
    button_init();
    led_init();

    // Start MQTT client
    mqtt_app_start();
}
