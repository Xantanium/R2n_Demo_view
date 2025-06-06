// #include "JSONLogger.hpp"
//
// /// @brief creates a new file, if the file already exists, a new file with a unique name is created. DO NOT CALL IN
// loop().
// /// @param filename name of the file to be created
// void rudra::JSONLogger::makeFile(const char* filename)
// {
//     _filename = filename;
//     fileExists = SD.exists(_filename);
//
//     if (fileExists) {
//         Serial.printf("File %s already exists, generating a unique name...\n", _filename);
//
//         char newFilename[50];
//         int i = 1;
//         while (true) {
//             sprintf(newFilename, "%s_%d.json", filename, i);
//             if (!SD.exists(newFilename)) {
//                 _filename = strdup(newFilename);
//                 break;
//             }
//             i++;
//         }
//
//         Serial.printf("New filename: %s\n", _filename);
//     }
//
//     _file = SD.open(_filename, FILE_WRITE);
//     if (!_file) {
//         Serial.println("Failed to create log file");
//         return;
//     }
//     _file.close();
// }
//
// /// @brief deletes specified file from SD card.
// /// @param filename file to be deleted
// void rudra::JSONLogger::deleteFile(const char* filename)
// {
//     if (SD.exists(filename)) {
//         SD.remove(filename);
//         Serial.printf("Deleted file: %s\n", filename);
//     } else {
//         Serial.println("File does not exist");
//     }
// }
//
// /// @brief adds metadata header to json document
// /// @param robotName name of the robot
// /// @param sessionDate date of current session
// void rudra::JSONLogger::addMetadata(const char* robotName, const char* sessionDate)
// {
//     JsonObject meta = doc["metadata"].to<JsonObject>();
//
//     meta["robotName"] = robotName;
//     meta["sessionDate"] = sessionDate;
//
//     _file = SD.open(_filename, FILE_WRITE);
//     if (!_file) {
//         Serial.println("Failed to open file for writing");
//         return;
//     }
//     _file.print("\"metadata\" : ");
//     serializeJsonPretty(meta, _file);
//     _file.println(",");
//     _file.close();
// }
//
// /// @brief clears json document used to hold the data before logging, does NOT clear the records from file.
// void rudra::JSONLogger::clearDoc()
// {
//     doc.clear();
// }
//
// /// @brief initiates the json document in file.
// void rudra::JSONLogger::beginJson()
// {
//     _file = SD.open(_filename, FILE_WRITE);
//     if (!_file) {
//         Serial.println("Failed to open file for writing");
//         return;
//     }
//
//     _file.println("{");
//
//     _file.close();
// }
//
// /// @brief ends the json document in file, call at the end of logging.
// void rudra::JSONLogger::endJson()
// {
//     _file = SD.open(_filename, FILE_WRITE);
//     if (!_file) {
//         Serial.println("Failed to open file for writing");
//         return;
//     }
//
//     serializeJson(doc, _file);
//     _file.println("}");
//     _file.println();
//     _file.close();
// }
//
// // /// @brief helper function to write JSON to file
// // void rudra::JSONLogger::writeToFile()
// // {
// //     _file = SD.open(_filename, FILE_WRITE);
// //     if (!_file) {
// //         Serial.println("Failed to open file for writing");
// //         return;
// //     }
//
// //     serializeJsonPretty(doc, _file);
// //     _file.println();
// //     _file.close();
// // }
