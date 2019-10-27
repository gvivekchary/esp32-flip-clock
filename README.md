# SBB Flip Clock

ESP32 gets time from NTP server and updates Flip Clock.

## Configuration in SDKCONFIG
- Set Wifi user and password (CONFIG_EXAMPLE_WIFI_SSID / CONFIG_EXAMPLE_WIFI_PASSWORD)
- NTP update interval: CONFIG_LWIP_SNTP_UPDATE_DELAY=3600000

## Configuration in `sbb_flip_clock`

### Used pins:
- `DE_PIN       23`
- `UART1_TX     22`

### Addresses of the individual modules (see addr on sticker)
- `ADDR_MINUTE  56`
- `ADDR_HOUR    7`

### Hardware

MAX481 - 5V supply
ESP32 UART1 TX provides 3.3V, which is sufficient to drive MAX481 input.
Don't connect RX pin as well (5V > 3.3V!)

