# DIY Davis Vantage Vue Sensor Suite receiver

This is the code of a project described in more details on my [blog](https://retameur.com/posts/vantage-receiver)


An ESP32-S3 program to receive data from a Davis Vantage Vue sensor suite (European version). Require an RFM69 RF tranceiver and AHT20 and BMP280 sensors.

| ESP32 | RFM69 | AHT20+BMP280 | Comment |
|-------|-------|--------------|---------|
| 15    | DIO0  |              |         |
| 16    | SCK   |              |         |
| 17    | MOSI  |              |         |
| 18    | MISO  |              |         |
|       | ANT   |              | Connect a wire of about 8.5 cm or a proper antenna |
| 3     |       |  SDA         |         |
| 9     |       |  SCL         |         |
| GND   | GND   |  GND         |         |
| 3V3   | 3.3V  |  VDD         |         |