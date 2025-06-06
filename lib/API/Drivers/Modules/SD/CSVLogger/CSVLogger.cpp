
// /// @brief Begins the file operations on the SD card
// /// @param csPin Chip select pin for the SD card
// /// @return true if SD card init successfully
// bool rudra::CSVLogger::begin(int csPin)
// {
//     if (!SD.begin(csPin)) {
//         Serial.println("SD card initialization failed");
//         return false;
//     }

//     fileExists = SD.exists(filename.c_str());

//     return true;
// }

// /// @brief Adds a column to the CSV file
// /// @param name column name
// /// @param type datatype of the column
// void rudra::CSVLogger::addColumn(String& name, String& type)
// {
//     headers.push_back({ name, type });
// }

// /// @brief initialize the file with headers.
// void rudra::CSVLogger::initFile()
// {
//     if (!fileExists)
//         return;

//     file = SD.open(filename.c_str(), FILE_WRITE);

//     if (file) {
//         for (size_t i = 0; i < headers.size(); i++) {
//             file.print(headers[i].name);
//             if (i != headers.size() - 1)
//                 file.print(",");
//         }
//         file.println();
//         file.close();
//     }

//     isHeaderWritten = true;
// }
