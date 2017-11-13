#ifndef ARDUINO_H
#define ARDUINO_H

typedef unsigned char uint8_t;
typedef char byte;
typedef char boolean;

int analogRead(int r);

#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define A6 6
#define A7 7

typedef __SIZE_TYPE__ size_t;

class HWSerial {
public:
	static void print(const char* t) {
	}
	static void print(int n) {
	}
	static void println(const char* t) {
	}
	static void println(int n) {
	}
	static int read(void) {
		return 0;
	}
};

extern HWSerial Serial;
extern void tone(int, int);
extern void noTone(int);

#endif
