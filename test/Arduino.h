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
  void clear() { buffer.clear(); }
};

extern HardwareSerial Serial1;

inline void delay(unsigned long) {}
inline void tone(int, unsigned int, unsigned long) {}
