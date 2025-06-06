namespace rudra {

/// \brief Reusable macro for defining loggable fields.
// Add fields to be logged here.
#define LOGGABLE_FIELDS                      \
    X(unsigned long, timeStamp, "timeStamp") \
    X(int, stickX, "stickX")                 \
    X(int, stickY, "stickY")                 \
    X(int, stickW, "stickW")                 \
    X(long, xCount, "xCount")                \
    X(long, yCount, "yCount")                \
    X(float, distX, "distX")                 \
    X(float, distY, "distY")                 \
    X(float, distW, "distW")

} // namespace rudra
