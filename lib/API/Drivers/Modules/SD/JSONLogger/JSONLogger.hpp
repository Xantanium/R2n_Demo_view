#pragma once

// #include "../LogConfig.h"
// #include <SD.h>
//
// #include <ctime>
//
// namespace rudra {
//
// class JSONLogger {
// private:
//     const char* _filename;
//     File _file;
//     JsonDocument doc; // JSON document with limited memory usage
//
//     bool fileExists; // Flag to check if file exists
//
//     // void writeToFile(); // Helper function to write JSON to file
//
// public:
//     JSONLogger()
//         : _filename(nullptr)
//         , fileExists(false)
//     {
//     }
//
//     void makeFile(const char* filename);
//     static void deleteFile(const char* filename);
//
//     inline bool begin();
//     void clearDoc();
//     void addMetadata(const char* robotName, const char* sessionDate);
//
//     void beginJson();
//     void endJson();
//
//     template <typename StructType> void logJSON(const StructType& data);
// };
//
// /// @brief initiates SD card usage
// /// @return SD init status (bool)
// inline bool JSONLogger::begin() { return SD.begin(BUILTIN_SDCARD); }
//
// /// @brief logs the data to the json document in file, keys are extracted from LOGGABLE_FIELDS macro from logConfig.h
// /// @tparam StructType user defined structure to hold the data
// /// @param data structure in the driver code to be logged
// template <typename StructType> void rudra::JSONLogger::logJSON(const StructType& data)
// {
//     _file = SD.open(_filename, FILE_WRITE);
//
//     if (!_file) {
//         Serial.println("Failed to open file");
//     }
//
//     JsonObject logs = doc["logs"].to<JsonObject>();
//
// #define X(type, name, jsonKey) logs[jsonKey] = data.name;
//     LOGGABLE_FIELDS
// #undef X
//
//     // _file.print("\"logs\" : ");
//     serializeJson(doc, _file);
//     _file.println(",");
// }
// } // namespace rudra
