| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

基于 ESP-IDF 驱动带有触摸功能的 SPI 屏（ST7789V+ST1615），LVGL运行的是SquareLine Studio 自带的手表 UI 示例。

## 硬件接线

接线可以修改宏定义来为 ``SPI`` 和 ``I2C`` 分配管脚。

可以参考本示例的管脚分配：

|屏幕管脚 | ESP32-S3 管脚 |
|-----|-------------|
|Backlight| NC          |
|SPI DC| GPIO 8      |
|SPI CS| GPIO 7      |
|SPI SCL| GPIO 6      |
|SPI SDA| GPIO 5      |
| SPI RESET| GPIO 9      |
|I2C TP_SCL| GPIO 16     |
|I2C TP_SDA| GPIO 15     |
|I2C TP_RST| GPIO 18/NC  |
|I2C TP_TINT| GPIO 17/NC  |

## 效果
![1](https://github.com/wx-dev/esp32-lcd-touch-st1615/assets/81051506/f979081b-f0bc-42d3-8f35-e6d5634b0104)
![2](https://github.com/wx-dev/esp32-lcd-touch-st1615/assets/81051506/92c7141b-04ec-4dbe-acd6-11b4db641328)
![3](https://github.com/wx-dev/esp32-lcd-touch-st1615/assets/81051506/9a871871-470a-4f23-a3a8-5018f35ea9d6)
