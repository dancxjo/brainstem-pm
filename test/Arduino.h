#pragma once
#include <stdint.h>
#include <vector>
#include <cstddef>

class HardwareSerial {
public:
  // Bytes written by the device under test
  std::vector<uint8_t> buffer;
  // Bytes to be read by the device under test
  std::vector<uint8_t> rx;

  void begin(unsigned long) {}

  size_t write(uint8_t b) {
    buffer.push_back(b);
    return 1;
  }
  size_t write(const uint8_t* data, size_t len) {
    buffer.insert(buffer.end(), data, data + len);
    return len;
  }
  int available() { return static_cast<int>(rx.size()); }
  int read() {
    if (rx.empty()) return -1;
    uint8_t b = rx.front();
    rx.erase(rx.begin());
    return b;
  }
  void clear() {
    buffer.clear();
    rx.clear();
  }
};

extern HardwareSerial Serial1;

inline void delay(unsigned long) {}
inline void tone(int, unsigned int, unsigned long) {}

// Minimal Arduino core shims for native tests
#ifndef HIGH
#define HIGH 0x1
#endif
#ifndef LOW
#define LOW  0x0
#endif
#ifndef INPUT
#define INPUT 0x0
#endif
#ifndef OUTPUT
#define OUTPUT 0x1
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP 0x2
#endif
#ifndef A0
#define A0 14
#endif

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return HIGH; }
inline int analogRead(int) { return 1023; }

// Interrupt stubs
#ifndef CHANGE
#define CHANGE 1
#endif
#ifndef RISING
#define RISING 2
#endif
#ifndef FALLING
#define FALLING 3
#endif
#ifndef NOT_AN_INTERRUPT
#define NOT_AN_INTERRUPT -1
#endif
inline int digitalPinToInterrupt(int) { return 0; }
inline void attachInterrupt(int, void (*)(void), int) {}

// simple millis() stub that advances with calls
inline unsigned long millis() {
  static unsigned long t = 0;
  t += 10;
  return t;
}

// simple pseudo-random generator compatible with Arduino's random(max)
inline long random(long max) {
  static unsigned long seed = 2463534242UL;
  // xorshift32
  seed ^= seed << 13;
  seed ^= seed >> 17;
  seed ^= seed << 5;
  return (long)(seed % (unsigned long)max);
}

// Minimal USB Serial stub for debug logs and passthrough tests
class USBSerial : public HardwareSerial {
public:
  void print(const char*) {}
  void println(const char*) {}
  void print(int) {}
  void println(int) {}
  void print(unsigned long) {}
  void println(unsigned long) {}
  void println() {}
};

static USBSerial Serial;

// LED helper macros as no-ops if referenced
#ifndef TXLED0
#define TXLED0 do {} while (0)
#endif
#ifndef TXLED1
#define TXLED1 do {} while (0)
#endif
#ifndef RXLED0
#define RXLED0 do {} while (0)
#endif
#ifndef RXLED1
#define RXLED1 do {} while (0)
#endif

// Brainstem UART build calls this to mark USB activity; provide a no-op stub for tests
inline void usbLinkActivity() {}
