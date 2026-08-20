// =============================================================================
// ESP32-C3 firmware: 2x AS5047P rotary encoders + ICM42688P IMU over SPI,
// published over WiFi via TCP at a fixed 100 Hz.
//
// Framework: native ESP-IDF (C++), NOT Arduino.
// Build: place in a standard ESP-IDF project as main/main.cpp, with
//        idf_component_register(SRCS "main.cpp" ...) in main/CMakeLists.txt.
//
// Pin map (from schematic):
//   SCLK        -> GPIO4
//   MOSI        -> GPIO6
//   MISO        -> GPIO5
//   CS_IMU      -> GPIO10   (ICM42688P, net "CS1")
//   CS_ENC1     -> GPIO0    (AS5047P #1, net "CS2")  ** boot-strap pin, see note below **
//   CS_ENC2     -> GPIO7    (AS5047P #2, net "CS3")
//
// NOTE on GPIO0: it's a boot-mode strap pin. It must not be pulled LOW at
// power-on/reset. In normal operation SPI CS idles HIGH so this is fine, but
// if you see random boot-into-download-mode issues, move CS_ENC1 to GPIO8 or
// GPIO9 (both appear unused/spare in your schematic) and update the #define.
// =============================================================================

#include <cstring>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "sensor_fw";

// ----------------------------------------------------------------------------
// User config
// ----------------------------------------------------------------------------
#define WIFI_SSID           "your_ssid"
#define WIFI_PASS           "your_password"

#define SERVER_IP           "192.168.1.100"   // PC / receiver IP
#define SERVER_PORT         5005

#define POLL_RATE_HZ        100               // sensor poll rate (30-100 Hz range; set within that)
#define PUBLISH_RATE_HZ     100                // TCP publish rate

// SPI pins
#define PIN_SCLK            GPIO_NUM_4
#define PIN_MOSI            GPIO_NUM_6
#define PIN_MISO            GPIO_NUM_5
#define PIN_CS_IMU          GPIO_NUM_10
#define PIN_CS_ENC1         GPIO_NUM_0        // see boot-strap note above
#define PIN_CS_ENC2         GPIO_NUM_7

#define SPI_HOST_USED       SPI2_HOST

// ----------------------------------------------------------------------------
// Shared sensor state
// ----------------------------------------------------------------------------
struct SensorSample {
    uint32_t t_us;
    uint16_t enc1_angle;   // 14-bit raw angle, 0-16383
    uint16_t enc2_angle;
    bool     enc1_err;
    bool     enc2_err;
    float    ax, ay, az;   // g
    float    gx, gy, gz;   // deg/s
};

static SensorSample g_latest = {};
static SemaphoreHandle_t g_dataMutex;

static spi_device_handle_t g_spiImu;
static spi_device_handle_t g_spiEnc1;
static spi_device_handle_t g_spiEnc2;

static volatile bool g_wifiConnected = false;

// =============================================================================
// WiFi
// =============================================================================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifiConnected = false;
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        g_wifiConnected = true;
        ESP_LOGI(TAG, "WiFi connected, got IP");
    }
}

static void wifi_init_sta()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr));

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// =============================================================================
// SPI init
// =============================================================================
static void spi_bus_setup()
{
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_MOSI;
    buscfg.miso_io_num = PIN_MISO;
    buscfg.sclk_io_num = PIN_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 32;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_USED, &buscfg, SPI_DMA_CH_AUTO));

    // ICM42688P: SPI mode 0, up to 24 MHz (we run conservatively at 8 MHz)
    spi_device_interface_config_t imu_cfg = {};
    imu_cfg.clock_speed_hz = 8 * 1000 * 1000;
    imu_cfg.mode = 0;
    imu_cfg.spics_io_num = PIN_CS_IMU;
    imu_cfg.queue_size = 4;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_USED, &imu_cfg, &g_spiImu));

    // AS5047P: SPI mode 1 (CPOL=0, CPHA=1), up to 10 MHz (run at 1 MHz for margin)
    spi_device_interface_config_t enc_cfg = {};
    enc_cfg.clock_speed_hz = 1 * 1000 * 1000;
    enc_cfg.mode = 1;
    enc_cfg.queue_size = 4;

    enc_cfg.spics_io_num = PIN_CS_ENC1;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_USED, &enc_cfg, &g_spiEnc1));

    enc_cfg.spics_io_num = PIN_CS_ENC2;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_USED, &enc_cfg, &g_spiEnc2));
}

// =============================================================================
// AS5047P (magnetic rotary encoder, 14-bit absolute angle over SPI)
// Frame: 16 bits. bit15=parity(even), bit14=RW(1=read), bits13:0=address/data.
// Reading a register takes two 16-bit transactions: send command, then clock
// out a NOP command to receive the previous response.
// =============================================================================
#define AS5047P_REG_ANGLECOM  0x3FFF   // angle with dynamic angle error comp.
#define AS5047P_REG_NOP       0x0000

static uint16_t as5047p_add_parity(uint16_t word)
{
    // even parity over bits 14:0, result stored in bit 15
    uint16_t p = word & 0x7FFF;
    p ^= p >> 8;
    p ^= p >> 4;
    p ^= p >> 2;
    p ^= p >> 1;
    uint16_t parity = p & 0x1;
    return (word & 0x7FFF) | (parity << 15);
}

static uint16_t as5047p_transfer(spi_device_handle_t dev, uint16_t cmd)
{
    uint16_t framed = as5047p_add_parity(cmd | 0x4000); // RW=1 (read)
    uint8_t tx[2] = { (uint8_t)(framed >> 8), (uint8_t)(framed & 0xFF) };
    uint8_t rx[2] = { 0, 0 };

    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    spi_device_polling_transmit(dev, &t);

    return (rx[0] << 8) | rx[1];
}

// Returns 14-bit angle (0-16383). Sets *err if the encoder reported EF (bit14).
static uint16_t as5047p_read_angle(spi_device_handle_t dev, bool *err)
{
    as5047p_transfer(dev, AS5047P_REG_ANGLECOM);     // issue read command
    uint16_t resp = as5047p_transfer(dev, AS5047P_REG_NOP); // clock out result

    *err = (resp & 0x4000) != 0;   // EF bit
    return resp & 0x3FFF;          // 14-bit angle data
}

// =============================================================================
// ICM42688P (IMU) - register access + init
// =============================================================================
#define ICM_REG_DEVICE_CONFIG   0x11
#define ICM_REG_PWR_MGMT0       0x4E
#define ICM_REG_GYRO_CONFIG0    0x4F
#define ICM_REG_ACCEL_CONFIG0   0x50
#define ICM_REG_ACCEL_DATA_X1   0x1F   // burst-read start: accel(6) + gyro(6) = 12 bytes

static void icm_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val }; // MSB=0 -> write
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    spi_device_polling_transmit(g_spiImu, &t);
}

static void icm_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t tx[1 + 32] = {0};
    uint8_t rx[1 + 32] = {0};
    tx[0] = reg | 0x80; // MSB=1 -> read

    spi_transaction_t t = {};
    t.length = (1 + len) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    spi_device_polling_transmit(g_spiImu, &t);

    memcpy(buf, &rx[1], len);
}

static void icm42688p_init()
{
    icm_write_reg(ICM_REG_DEVICE_CONFIG, 0x01); // soft reset
    vTaskDelay(pdMS_TO_TICKS(10));

    // Accel + gyro -> Low Noise mode
    icm_write_reg(ICM_REG_PWR_MGMT0, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Gyro: FS=2000dps, ODR=1kHz  (adjust per datasheet table if you want different range)
    icm_write_reg(ICM_REG_GYRO_CONFIG0, 0x06);
    // Accel: FS=16g, ODR=1kHz
    icm_write_reg(ICM_REG_ACCEL_CONFIG0, 0x06);

    vTaskDelay(pdMS_TO_TICKS(5));
}

static void icm42688p_read(float *ax, float *ay, float *az,
                            float *gx, float *gy, float *gz)
{
    uint8_t raw[12];
    icm_read_regs(ICM_REG_ACCEL_DATA_X1, raw, 12);

    int16_t ax_raw = (raw[0] << 8) | raw[1];
    int16_t ay_raw = (raw[2] << 8) | raw[3];
    int16_t az_raw = (raw[4] << 8) | raw[5];
    int16_t gx_raw = (raw[6] << 8) | raw[7];
    int16_t gy_raw = (raw[8] << 8) | raw[9];
    int16_t gz_raw = (raw[10] << 8) | raw[11];

    // Sensitivity for FS=16g -> 2048 LSB/g ; FS=2000dps -> 16.4 LSB/(deg/s)
    const float ACCEL_SENS = 2048.0f;
    const float GYRO_SENS  = 16.4f;

    *ax = ax_raw / ACCEL_SENS;
    *ay = ay_raw / ACCEL_SENS;
    *az = az_raw / ACCEL_SENS;
    *gx = gx_raw / GYRO_SENS;
    *gy = gy_raw / GYRO_SENS;
    *gz = gz_raw / GYRO_SENS;
}

// =============================================================================
// Sensor poll task (encoders + IMU), runs at POLL_RATE_HZ
// =============================================================================
static void sensor_task(void *arg)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / POLL_RATE_HZ);

    for (;;) {
        SensorSample s;
        s.t_us = (uint32_t)esp_timer_get_time();

        s.enc1_angle = as5047p_read_angle(g_spiEnc1, &s.enc1_err);
        s.enc2_angle = as5047p_read_angle(g_spiEnc2, &s.enc2_err);

        icm42688p_read(&s.ax, &s.ay, &s.az, &s.gx, &s.gy, &s.gz);

        xSemaphoreTake(g_dataMutex, portMAX_DELAY);
        g_latest = s;
        xSemaphoreGive(g_dataMutex);

        vTaskDelayUntil(&lastWake, period);
    }
}

// =============================================================================
// TCP publish task, fixed PUBLISH_RATE_HZ, sends binary packets
// =============================================================================
#pragma pack(push, 1)
struct Packet {
    uint32_t seq;
    uint32_t t_us;
    uint16_t enc1_angle;
    uint16_t enc2_angle;
    uint8_t  enc1_err;
    uint8_t  enc2_err;
    float    ax, ay, az;
    float    gx, gy, gz;
};
#pragma pack(pop)

static int connect_tcp()
{
    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(SERVER_PORT);
    dest.sin_addr.s_addr = inet_addr(SERVER_IP);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        return -1;
    }

    int nodelay = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    struct timeval sndtimeo = { .tv_sec = 0, .tv_usec = 20000 }; // 20ms send timeout
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &sndtimeo, sizeof(sndtimeo));

    if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) != 0) {
        ESP_LOGW(TAG, "connect() failed: errno %d", errno);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "TCP connected to %s:%d", SERVER_IP, SERVER_PORT);
    return sock;
}

static void publish_task(void *arg)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / PUBLISH_RATE_HZ);

    int sock = -1;
    uint32_t seq = 0;
    TickType_t lastReconnectAttempt = 0;

    for (;;) {
        if (!g_wifiConnected) {
            vTaskDelayUntil(&lastWake, period);
            continue;
        }

        if (sock < 0) {
            TickType_t now = xTaskGetTickCount();
            if (now - lastReconnectAttempt > pdMS_TO_TICKS(1000)) {
                sock = connect_tcp();
                lastReconnectAttempt = now;
            }
            vTaskDelayUntil(&lastWake, period);
            continue;
        }

        SensorSample s;
        xSemaphoreTake(g_dataMutex, portMAX_DELAY);
        s = g_latest;
        xSemaphoreGive(g_dataMutex);

        Packet pkt;
        pkt.seq = seq++;
        pkt.t_us = s.t_us;
        pkt.enc1_angle = s.enc1_angle;
        pkt.enc2_angle = s.enc2_angle;
        pkt.enc1_err = s.enc1_err;
        pkt.enc2_err = s.enc2_err;
        pkt.ax = s.ax; pkt.ay = s.ay; pkt.az = s.az;
        pkt.gx = s.gx; pkt.gy = s.gy; pkt.gz = s.gz;

        int sent = send(sock, &pkt, sizeof(pkt), 0);
        if (sent < 0) {
            ESP_LOGW(TAG, "send() failed: errno %d, reconnecting", errno);
            close(sock);
            sock = -1;
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

// =============================================================================
// main
// =============================================================================
extern "C" void app_main(void)
{
    g_dataMutex = xSemaphoreCreateMutex();

    wifi_init_sta();
    spi_bus_setup();
    icm42688p_init();

    // wait for wifi before starting tasks that need it (sensor task can start immediately)
    xTaskCreatePinnedToCore(sensor_task, "sensor_task", 4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(publish_task, "publish_task", 4096, nullptr, 5, nullptr, 1);

    ESP_LOGI(TAG, "Firmware started: poll=%dHz publish=%dHz", POLL_RATE_HZ, PUBLISH_RATE_HZ);
}
