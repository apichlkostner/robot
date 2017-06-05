ARDUINO_DIR            = /usr/share/arduino
TARGET                 = robot
ARDUINO_LIBS           =
MCU                    = atmega328p
F_CPU                  = 16000000
ARDUINO_PORT           = /dev/ttyUSB0
AVRDUDE_ARD_BAUDRATE   = 115200
AVRDUDE_ARD_PROGRAMMER = arduino
CPPFLAGS              = -fdiagnostics-color=always -IRedBotLib
LOCAL_CPP_SRCS         ?= $(wildcard ./RedBotLib/*.cpp *.cpp)

include /usr/share/arduino/Arduino.mk
