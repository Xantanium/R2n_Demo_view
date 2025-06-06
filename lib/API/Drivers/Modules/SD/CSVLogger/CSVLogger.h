// #include "logger.h"

// namespace rudra {
// class CSVLogger : public Logger {
// private:
//     File file;
//     String filename;

//     struct column {
//         String name;
//         String type;
//     };

//     // For column & respective datatype storage
//     std::vector<column> headers;

//     bool fileExists = 0;
//     bool isHeaderWritten = false;

// public:
//     CSVLogger(const String& filename)
//         : filename(filename) { };

//     bool begin(int csPin); // CS pin for SD card
//     void addColumn(String name, String type);
//     void initFile();

//     template <typename... args>
//     void log(args... data);

// private:
//     template <typename... args>
//     void writeRow(args... data);

//     void writeRow() { };
// };

// /// @brief logs data to the file
// /// @tparam ...args variadic template for accepting multiple data types
// /// @param ...data values corresponding to the columns
// template <typename... args>
// inline void CSVLogger::log(args... data)
// {
//     file = SD.open(filename, FILE_WRITE);

//     if (file) {
//         writeRow(args);

//         file.println();
//         file.close();
//     } else {
//         Serial.println("Error opening file");
//         return;
//     }
// }

// /// @brief appends the row to csv
// /// @tparam ...args variadic template for accepting multiple data types
// /// @param ...data values corresponding to columns
// template <typename... args>
// inline void CSVLogger::writeRow(args... data)
// {
//     ((file.print(data), file.print(",")), ...);
// }

// } // namespace rudra

/* namespace rudra {

// template for struct fields extraction
template <typename StructType, typename Func>
void forEachField(StructType& data, Func&& func)
{
#define APPLY_FIELD(name, type) func(#name, data.name)
    LOG_FIELDS(APPLY_FIELD);
}

// Main logger class
template <typename T>
class CSVLogger : public Logger {
private:
    const char* _filename; // name of .csv
    File _file; // .csv
    bool fileExists = false;
    T& _dataStruct;

public:
    CSVLogger(const char* filename, T& dataStruct)
        : _filename(filename)
        , _dataStruct(dataStruct)
    {
    }

    bool begin(uint8_t csPin);

    void addColumns();

    void log(T& data);
};

template <typename T>
inline bool CSVLogger<T>::begin(uint8_t csPin)
{
    if (!SD.begin(csPin))
        return false;

    fileExists = SD.exists(_filename);
    return true;
}

template <typename T>
inline void CSVLogger<T>::addColumns()
{
    if (fileExists)
        return;

    _file = SD.open(_filename, FILE_WRITE);

    if (_file) {
        forEachField(
            _dataStruct,
            [&](const char* name, auto&) {
                _file.print(name);
                _file.print(",");
            });

        _file.println();
        _file.close();
    }
}

template <typename T>
inline void CSVLogger<T>::log(T& data)
{
    _file = SD.open(_filename, FILE_WRITE);

    if (_file) {
        forEachField(data, [&](const char*, auto& value) {
            _file.print(value);
            _file.print(",");
        });
        _file.println();
        _file.close();
    }
}
} // namespace rudra */
#pragma once

#include "ArxContainer.h"
#include <../LogConfig.h>
#include <SD.h>

namespace rudra {
class CSVLogger {
private:
    File _file;
    const char* _filename;

    struct Headers {
        const char* name;
        const char* type;
    };

public:
    CSVLogger(/* args */);
    ~CSVLogger();
};

CSVLogger::CSVLogger(/* args */) { }

CSVLogger::~CSVLogger() { }

} // namespace rudra
