////////////////////////////////
// Default Logging Parameters //
////////////////////////////////
#define ENABLE_TIME_LOG           true
#define ENABLE_CALCULATED_LOG     true
#define ENABLE_ACCEL_LOG          true
#define ENABLE_GYRO_LOG           true
#define ENABLE_MAG_LOG            true
#define ENABLE_QUAT_LOG           false
#define ENABLE_EULER_LOG          false
#define ENABLE_HEADING_LOG        false

////////////////////////
// Serial Port Config //
////////////////////////
#define ENABLE_UART_LOGGING       true
// Select the Serial port to log to. Either SERIAL_PORT_USBVIRTUAL
// or LOG_PORT SERIAL_PORT_HARDWARE (SerialUSB or Serial1)
#define LOG_PORT                  SERIAL_PORT_USBVIRTUAL
#define SERIAL_BAUD_RATE          921600 // Serial port baud

////////////////
// LED Config //
////////////////
#define HW_LED_PIN                13   // LED attached to pin 13
#define UART_BLINK_RATE           1000 // Blink rate when only UART logging

/////////////////////////
// IMU Default Configs //
/////////////////////////
// Note: Some of these params can be overwritten using serial
//  commands. These are just defaults on initial programming
#define DMP_SAMPLE_RATE           200 // Logging/DMP sample rate(4-200 Hz)
#define IMU_COMPASS_SAMPLE_RATE   200 // Compass sample rate (4-100 Hz)
#define IMU_AG_SAMPLE_RATE        200 // Accel/gyro sample rate Must be between 4Hz and 1kHz
#define IMU_GYRO_FSR              2000 // Gyro full-scale range (250, 500, 1000, or 2000)
#define IMU_ACCEL_FSR             16 // Accel full-scale range (2, 4, 8, or 16)
#define IMU_AG_LPF                42 // Accel/Gyro LPF corner frequency (5, 10, 20, 42, 98, or 188 Hz)
#define ENABLE_GYRO_CALIBRATION   true
#define COMPASS_ENABLED           false

///////////////////////
// SD Logging Config //
///////////////////////
#define ENABLE_SD_LOGGING         true // Default SD logging (can be changed via serial menu)
#define LOG_FILE_INDEX_MAX        999 // Max number of "logXXX.txt" files
#define LOG_FILE_PREFIX           "log"  // Prefix name for log files
#define LOG_FILE_SUFFIX           "txt"  // Suffix name for log files
#define SD_MAX_FILE_SIZE          5000000 // 5MB max file size, increment to next file before surpassing
#define SD_LOG_WRITE_BUFFER_SIZE  1024 // Experimentally tested to produce 100Hz logs

//////////////////////////
// Hardware Definitions //
//////////////////////////
// Danger - don't change unless using a different platform
#define MPU9250_INT_PIN           4
#define SD_CHIP_SELECT_PIN        38
#define MPU9250_INT_ACTIVE        LOW
