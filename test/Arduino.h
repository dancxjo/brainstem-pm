#pragma once
#include <stdint.h>
#include <vector>
#include <cstddef>

class HardwareSerial {
public:
  std::vector<uint8_t> buffer;
  void begin(unsigned long) {}
  size_t write(uint8_t b) {
    buffer.push_back(b);
    return 1;
  }
  size_t write(const uint8_t* data, size_t len) {
    buffer.insert(buffer.end(), data, data + len);
    return len;
  }
  int available() { return 0; }
  int read() { return -1; }
  void clear() { buffer.clear(); }
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
#ifndef A0
#define A0 14
#endif

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return HIGH; }
inline int analogRead(int) { return 1023; }

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
