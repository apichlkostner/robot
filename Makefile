ARDUINO_DIR            = /usr/share/arduino
TARGET                 = test
ARDUINO_LIBS           =
MCU                    = atmega328p
F_CPU                  = 16000000
ARDUINO_PORT           = /dev/ttyUSB0
AVRDUDE_ARD_BAUDRATE   = 115200
AVRDUDE_ARD_PROGRAMMER = arduino
USER_LIB_PATH          :=

include /usr/share/arduino/Arduino.mk
