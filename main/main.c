#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "ca.h" // Include file chứng chỉ
#include "minmea.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "i2cdev.h"
#include "ina219.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define WIFI_SSID      "chinhchoe"
#define WIFI_PASS      "1234567890"
#define MQTT_BROKER    "wss://mqttws.pmc18nd.id.vn:443"
#define MQTT_TOPIC     "test/topic"
#define PMS_UART_PORT      UART_NUM_2
#define PMS_UART_RX_PIN    4
#define PMS_UART_TX_PIN    -1
#define PMS_UART_BUF_SIZE  1024
#define GPS_UART_PORT      UART_NUM_1
#define GPS_UART_RX_PIN    17
#define GPS_UART_TX_PIN    5
#define GPS_UART_BUF_SIZE  512
#define SD_PIN_NUM_MISO 19
#define SD_PIN_NUM_MOSI 23
#define SD_PIN_NUM_CLK  18
#define SD_PIN_NUM_CS   16
#define SD_MOUNT_POINT  "/sdcard"
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_IO      25
#define I2C_SCL_IO      33
#define INA219_ADDR     0x40
#define SHUNT_OHMS      0.10f
#define CURRENT_SIGN    (+1.0f)

static const char *TAG = "DATN_MAIN";
static esp_mqtt_client_handle_t mqtt_client = NULL;

// Biến toàn cục volatile cho GPS và PMS
volatile float gps_lat = 0, gps_lon = 0;
volatile bool gps_valid = false;
volatile int pm25_value = -1;
volatile bool pm25_valid = false;

// Forward decl
static void task_sensor_loop(void *arg);
static void upload_csv_on_reconnect(void);

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying WiFi connection...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static esp_err_t wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &wifi_event_handler,
                                        NULL,
                                        &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler,
                                        NULL,
                                        &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    ESP_LOGI(TAG, "wifi_init_sta finished.");
    return ESP_OK;
}

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            upload_csv_on_reconnect(); // Upload dữ liệu cũ khi có mạng lại
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        default:
            break;
    }
}

static void mqtt_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER,
        .broker.verification.certificate = mqtt_ca_cert,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

// NTP
static void ntp_init_and_wait(void) {
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_init();
    const int64_t WAIT_MAX_MS = 10000;
    int64_t t0 = esp_timer_get_time() / 1000;
    while (true) {
        time_t now = 0;
        time(&now);
        if (now > 1600000000) {
            ESP_LOGI(TAG, "NTP time synced: %ld", (long)now);
            break;
        }
        if ((esp_timer_get_time()/1000 - t0) > WAIT_MAX_MS) {
            ESP_LOGW(TAG, "NTP sync timeout, continue anyway");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static int64_t get_timestamp_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000LL + (tv.tv_usec / 1000);
}

// UART PMS
void uart_pms_init() {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(PMS_UART_PORT, PMS_UART_BUF_SIZE*2, 0, 0, NULL, 0);
    uart_param_config(PMS_UART_PORT, &uart_config);
    uart_set_pin(PMS_UART_PORT, PMS_UART_TX_PIN, PMS_UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int pms5003_read_pm25() {
    uint8_t data[32];
    int pm25 = -1;
    while (1) {
        int len = uart_read_bytes(PMS_UART_PORT, data, 1, pdMS_TO_TICKS(1000));
        if (len == 1 && data[0] == 0x42) {
            len = uart_read_bytes(PMS_UART_PORT, data + 1, 31, pdMS_TO_TICKS(100));
            if (len == 31 && data[1] == 0x4D) {
                uint16_t sum = 0;
                for (int i = 0; i < 30; i++) sum += data[i];
                uint16_t checksum = (data[30] << 8) | data[31];
                if (sum == checksum) {
                    pm25 = (data[12] << 8) | data[13];
                }
            }
            break;
        }
    }
    return pm25;
}

// UART GPS
void uart_gps_init() {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(GPS_UART_PORT, GPS_UART_BUF_SIZE*2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_PORT, &uart_config);
    uart_set_pin(GPS_UART_PORT, GPS_UART_TX_PIN, GPS_UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

bool gps_get_latlon(float *lat, float *lon) {
    char line[128];
    for (int tries = 0; tries < 10; tries++) {
        int len = uart_read_bytes(GPS_UART_PORT, (uint8_t*)line, sizeof(line)-1, pdMS_TO_TICKS(500));
        if (len <= 0) continue;
        line[len] = 0;
        char *start = line;
        while (start && *start) {
            char *end = strchr(start, '\n');
            if (end) *end = 0;
            if (minmea_sentence_id(start, false) == MINMEA_SENTENCE_RMC) {
                struct minmea_sentence_rmc frame;
                if (minmea_parse_rmc(&frame, start) && frame.valid) {
                    *lat = minmea_tocoord(&frame.latitude);
                    *lon = minmea_tocoord(&frame.longitude);
                    return true;
                }
            }
            if (!end) break;
            start = end + 1;
        }
    }
    return false;
}

// INA219
static ina219_t g_ina = {0};
static esp_err_t ina219_init_all(void) {
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(ina219_init_desc(&g_ina, INA219_ADDR, I2C_PORT, I2C_SDA_IO, I2C_SCL_IO));
    ESP_ERROR_CHECK(ina219_init(&g_ina));
    ESP_ERROR_CHECK(ina219_calibrate(&g_ina, SHUNT_OHMS));
    return ESP_OK;
}

static esp_err_t ina219_read_current(float *current_out) {
    float current = 0.0f;
    esp_err_t err = ina219_get_current(&g_ina, &current);
    if (err != ESP_OK) return err;
    current *= CURRENT_SIGN;
    *current_out = current;
    return ESP_OK;
}

// SD card
static sdmmc_card_t *g_sdcard = NULL;
static esp_err_t sdcard_init(void) {
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_PIN_NUM_MOSI,
        .miso_io_num = SD_PIN_NUM_MISO,
        .sclk_io_num = SD_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA));

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_PIN_NUM_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = true,
        .max_files = 10,
        .allocation_unit_size = 4 * 1024
    };
    esp_err_t ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_cfg, &g_sdcard);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "SD mounted. Card: %s", g_sdcard->cid.name);
    return ESP_OK;
}

// Upload CSV khi có mạng lại
static void upload_csv_on_reconnect(void) {
    FILE *f = fopen("/sdcard/datn.csv", "r");
    if (!f) {
        ESP_LOGI(TAG, "Không tìm thấy file datn.csv để upload");
        return;
    }
    char line[256];
    int sent = 0;
    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, "timestamp") == line) continue; // Bỏ header
        if (mqtt_client) {
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, line, 0, 1, 0);
            sent++;
        }
    }
    fclose(f);
    ESP_LOGI(TAG, "Đã upload %d dòng dữ liệu cũ lên server", sent);
    remove("/sdcard/datn.csv"); // Xóa file sau upload
    ESP_LOGI(TAG, "Đã xóa file datn.csv sau khi upload");
}

// Task đọc GPS liên tục
void task_gps(void *arg) {
    char line[128];
    while (1) {
        int len = uart_read_bytes(GPS_UART_PORT, (uint8_t*)line, sizeof(line)-1, pdMS_TO_TICKS(200));
        if (len > 0) {
            line[len] = 0;
            char *start = line;
            while (start && *start) {
                char *end = strchr(start, '\n');
                if (end) *end = 0;
                if (minmea_sentence_id(start, false) == MINMEA_SENTENCE_RMC) {
                    struct minmea_sentence_rmc frame;
                    if (minmea_parse_rmc(&frame, start) && frame.valid) {
                        gps_lat = minmea_tocoord(&frame.latitude);
                        gps_lon = minmea_tocoord(&frame.longitude);
                        gps_valid = true;
                    }
                }
                if (!end) break;
                start = end + 1;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Task đọc PMS liên tục
void task_pms(void *arg) {
    while (1) {
        int val = pms5003_read_pm25();
        if (val >= 0) {
            pm25_value = val;
            pm25_valid = true;
        } else {
            pm25_valid = false;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Task chính: ghi file và publish
static void task_sensor_loop(void *arg) {
    char msg[256];
    while (1) {
        int64_t ts = get_timestamp_ms();
        float current = 0.0f;
        if (ina219_read_current(&current) != ESP_OK) {
            current = 0.0f;
        }

        // Ghi file CSV
        FILE *f = fopen("/sdcard/datn.csv", "a");
        if (f) {
            fprintf(f, "%lld,%d,%.3f,%.6f,%.6f\n", (long long)ts, pm25_valid ? pm25_value : -1, current, gps_valid ? gps_lat : 0.0f, gps_valid ? gps_lon : 0.0f);
            fclose(f);
            ESP_LOGI(TAG, "Data appended to CSV");
        }

        // Publish MQTT
        if (pm25_valid) {
            if (gps_valid) {
                snprintf(msg, sizeof(msg), "{\"timestamp\":%lld,\"pm25\":%d,\"current\":%.3f,\"lat\":%.6f,\"lon\":%.6f}", (long long)ts, pm25_value, current, gps_lat, gps_lon);
            } else {
                snprintf(msg, sizeof(msg), "{\"timestamp\":%lld,\"pm25\":%d,\"current\":%.3f,\"gps\":\"invalid\"}", (long long)ts, pm25_value, current);
            }
        } else {
            snprintf(msg, sizeof(msg), "{\"timestamp\":%lld,\"pm25\":\"invalid\",\"current\":%.3f}", (long long)ts, current);
        }
        ESP_LOGI(TAG, "Publish: %s", msg);
        if (mqtt_client) {
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, msg, 0, 1, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// App main
void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_ERROR_CHECK(nvs_flash_init());

    // SD card trước
    esp_err_t sd_err = sdcard_init();
    if (sd_err != ESP_OK) {
        ESP_LOGW(TAG, "Không thể kết nối thẻ SD, tiếp tục chạy không lưu dữ liệu lên thẻ!");
    }

    // WiFi + NTP
    ESP_ERROR_CHECK(wifi_init_sta());
    ntp_init_and_wait();

    // UART
    uart_pms_init();
    uart_gps_init();

    // I2C + INA219
    ESP_ERROR_CHECK(ina219_init_all());

    // MQTT
    mqtt_start();

    // Tasks
    xTaskCreate(task_gps, "task_gps", 2048, NULL, 10, NULL);
    xTaskCreate(task_pms, "task_pms", 2048, NULL, 10, NULL);
    xTaskCreate(task_sensor_loop, "task_sensor_loop", 6144, NULL, 10, NULL);
}