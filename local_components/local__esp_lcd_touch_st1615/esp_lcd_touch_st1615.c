#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"

#define POINT_NUM_MAX       (1)

#define DATA_START_REG      (0x10)
#define CONTACT_COUNT_MAX   (0x3F)
#define  FIRMWARE_VERSION   (0x00)

static const char *TAG = "ST1615";

static esp_err_t read_data(esp_lcd_touch_handle_t tp);

static bool get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num,
                   uint8_t max_point_num);

static esp_err_t del(esp_lcd_touch_handle_t tp);

static esp_err_t i2c_read_bytes(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len);

static esp_err_t reset(esp_lcd_touch_handle_t tp);

static esp_err_t read_firmware_version(esp_lcd_touch_handle_t tp);

esp_err_t esp_lcd_touch_new_i2c_st1615(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config,
                                       esp_lcd_touch_handle_t *tp) {
    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_ARG, TAG, "Invalid io");
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Invalid config");
    ESP_RETURN_ON_FALSE(tp, ESP_ERR_INVALID_ARG, TAG, "Invalid touch handle");

    /* Prepare main structure */
    esp_err_t ret = ESP_OK;
    esp_lcd_touch_handle_t st1615 = calloc(1, sizeof(esp_lcd_touch_t));
    ESP_GOTO_ON_FALSE(st1615, ESP_ERR_NO_MEM, err, TAG, "Touch handle malloc failed");

    /* Communication interface */
    st1615->io = io;
    /* Only supported callbacks are set */
    st1615->read_data = read_data;
    st1615->get_xy = get_xy;
    st1615->del = del;
    /* Mutex */
    st1615->data.lock.owner = portMUX_FREE_VAL;
    /* Save config */
    memcpy(&st1615->config, config, sizeof(esp_lcd_touch_config_t));

    /* Prepare pin for touch interrupt */
    if (st1615->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
                .mode = GPIO_MODE_INPUT,
                .intr_type = GPIO_INTR_NEGEDGE,
                .pin_bit_mask = BIT64(st1615->config.int_gpio_num)
        };
        ESP_GOTO_ON_ERROR(gpio_config(&int_gpio_config), err, TAG, "GPIO intr config failed");

        /* Register interrupt callback */
        if (st1615->config.interrupt_callback) {
            esp_lcd_touch_register_interrupt_callback(st1615, st1615->config.interrupt_callback);
        }
    }
    /* Prepare pin for touch controller reset */
    if (st1615->config.rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_gpio_config = {
                .mode = GPIO_MODE_OUTPUT,
                .pin_bit_mask = BIT64(st1615->config.rst_gpio_num)
        };
        ESP_GOTO_ON_ERROR(gpio_config(&rst_gpio_config), err, TAG, "GPIO reset config failed");
    }
    /* Reset controller */
    ESP_GOTO_ON_ERROR(reset(st1615), err, TAG, "Reset failed");
    /* Read firmware version */
    ESP_GOTO_ON_ERROR(read_firmware_version(st1615), err, TAG, "Read firmware version failed");
    *tp = st1615;

    return ESP_OK;
    err:
    if (st1615) {
        del(st1615);
    }
    ESP_LOGE(TAG, "Initialization failed!");
    return ret;
}

static esp_err_t read_data(esp_lcd_touch_handle_t tp) {
    typedef struct {
        uint8_t y_h: 3;
        uint8_t reserved: 1;
        uint8_t x_h: 3;
        uint8_t valid: 1;
        uint8_t x_l;
        uint8_t y_l;
        uint8_t reserved2;
    } xy_data_t;
    typedef struct {
        uint8_t gesture_type: 4;
        uint8_t reserved: 1;
        uint8_t water_flag: 1;
        uint8_t proximity_flag: 1;
        uint8_t reserved2: 1;
        uint8_t keys;
        xy_data_t xy_data[10];
    } stx_report_data_t;

    stx_report_data_t point;
    ESP_RETURN_ON_ERROR(i2c_read_bytes(tp, DATA_START_REG, (uint8_t *) &point, sizeof(stx_report_data_t)), TAG,
                        "I2C read failed");

//    uint8_t contact_count_max = 2;
//    ESP_RETURN_ON_ERROR(i2c_read_bytes(tp, CONTACT_COUNT_MAX, &contact_count_max, 1), TAG, "I2C read failed2");
    portENTER_CRITICAL(&tp->data.lock);
    /* Fill all coordinates */
    uint8_t count = 0;
    for (int i = 0; i < 2; i++) {
        if (point.xy_data[i].valid) {
            tp->data.coords[i].x = point.xy_data[i].x_h << 8 | point.xy_data[i].x_l;
            tp->data.coords[i].y = point.xy_data[i].y_h << 8 | point.xy_data[i].y_l;
            count++;
        }
    }
    tp->data.points = count;
    portEXIT_CRITICAL(&tp->data.lock);
    return ESP_OK;
}

static bool get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num,
                   uint8_t max_point_num) {
    portENTER_CRITICAL(&tp->data.lock);
    /* Count of points */
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);
    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength) {
            strength[i] = tp->data.coords[i].strength;
        }
    }
    /* Invalidate */
    tp->data.points = 0;
    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t del(esp_lcd_touch_handle_t tp) {
    /* Reset GPIO pin settings */
    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
    }
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }
    /* Release memory */
    free(tp);

    return ESP_OK;
}

static esp_err_t reset(esp_lcd_touch_handle_t tp) {
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, tp->config.levels.reset), TAG,
                            "GPIO set level failed");
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, !tp->config.levels.reset), TAG,
                            "GPIO set level failed");
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    return ESP_OK;
}

static esp_err_t read_firmware_version(esp_lcd_touch_handle_t tp) {
    uint8_t firmware_version;
    ESP_RETURN_ON_ERROR(i2c_read_bytes(tp, FIRMWARE_VERSION, &firmware_version, 1), TAG, "I2C read failed");
    ESP_LOGI(TAG, "FIRMWARE_VERSION : %d", firmware_version);
    return ESP_OK;
}

static esp_err_t i2c_read_bytes(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len) {
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "Invalid data");

    return esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
}
