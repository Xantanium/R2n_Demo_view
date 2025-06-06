#pragma once

// storage operations
#include <SD.h>
#include <SPI.h>

#include <tuple>
#include <vector>

namespace rudra {

// made for ability to expand the logger to support other storaget types
class Logger {
public:
    Logger(/* args */);
    ~Logger();
};

} // namespace rudra
