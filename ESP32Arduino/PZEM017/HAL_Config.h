#pragma once

#if CONFIG_IDF_TARGET_ESP32
#define RS485_SERIAL      Serial2

#define MAX485_DE         GPIO_NUM_27          // Define DE Pin to Arduino pin. Connect DE Pin of Max485 converter module
#define MAX485_RE         GPIO_NUM_26          // Define RE Pin to Arduino pin. Connect RE Pin of Max485 converter module

#define RS485_SERIAL_RX   GPIO_NUM_25
#define RS485_SERIAL_TX   GPIO_NUM_14

#else
#error Pines de control no definidos para board objetivo!
#endif

#define RK520_MODBUS_ADDR 0x01
#define RK520_MODBUS_BAUD 9600
