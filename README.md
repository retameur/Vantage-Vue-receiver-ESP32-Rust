# DIY Davis Vantage Vue Sensor Suite receiver

This is the code of a project described in more details on my [blog](https://retameur.com/posts/vantage-receiver)


An ESP32-S3 program to receive data from a Davis Vantage Vue sensor suite (European version). Require an RFM69 RF tranceiver and AHT20 and BMP280 sensors.

| ESP32 | RFM69 | AHT20+BMP280 | Comment |
|-------|-------|--------------|---------|
| 8     | DIO0  |              |         |
| 6     | NSS   |              |         |
| 7     | SCK   |              |         |
| 15    | MOSI  |              |         |
| 16    | MISO  |              |         |
|       | ANT   |              | Connect a wire of about 8.5 cm or a proper antenna |
| 14    |       |  SDA         |         |
| 21    |       |  SCL         |         |
| GND   | GND   |  GND         |         |
| 3V3   | 3.3V  |  VDD         |         |