/******************************************************************************
SparkFun 9DoF Razor M0 Example Firmware
Jim Lindblom @ SparkFun Electronics
Original creation date: November 22, 2016
https://github.com/sparkfun/9DOF_Razor_IMU/Firmware

This example firmware for the SparkFun 9DoF Razor IMU M0 
demonstrates how to grab accelerometer, gyroscope, magnetometer,
and quaternion values from the MPU-9250's digital motion processor
(DMP). It prints those values to a serial port and, if a card is
present, an SD card.

Values printed can be configured using the serial port. Settings
can be modified using the included "config.h" file.

Resources:
SparkFun MPU9250-DMP Arduino Library:
  https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library
FlashStorage Arduino Library
  https://github.com/cmaglie/FlashStorage

Development environment specifics:
  Firmware developed using Arduino IDE 1.6.12

Hardware:
  SparkFun 9DoF Razor IMU M0 (SEN-14001)
  https://www.sparkfun.com/products/14001
******************************************************************************/
// MPU-9250 Digital Motion Processing (DMP) Library
#include <SparkFunMPU9250-DMP.h>
// SD Library manages file and hardware control
#include <SD.h>
// config.h manages default logging parameters and can be used
// to adjust specific parameters of the IMU
#include "config.h"
#include <FlashStorage.h>
#include "SparkFun_MS5637_Arduino_Library.h"
#include "BoardData.h"
#include "circbuffer.h"
#include "sparkfun_mpl31125.h"
#include <Wire.h>



#define LOOP_TIME 2 //ms
uint32_t last_loop_start = 0;
uint32_t lastlooplogged = 0;

bool sdLogBoardData(BoardData* bd, uint16_t count);

MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class
BoardData databufferbackingarray[512]; //Keep 2^9 previous iterations of data in memory, for launch detect and whatnot
CircBuffer<BoardData> databuffer(databufferbackingarray, 512);

/////////////////////////////
// Logging Control Globals //
/////////////////////////////
unsigned short accelFSR = IMU_ACCEL_FSR;
unsigned short gyroFSR = IMU_GYRO_FSR;
unsigned short fifoRate = DMP_SAMPLE_RATE;

/////////////////////
// SD Card Globals //
/////////////////////
bool sdCardPresent = false; // Keeps track of if SD card is plugged in
String logFileName; // Active logging file
String logFileBuffer; // Buffer for logged data. Max is set in config

MS5637 psensor;
File logFile;

bool readingPressure = false;

///////////////////////
// LED Blink Control //
///////////////////////
//bool ledState = false;
uint32_t lastBlink = 0;

int pin_ovf_led = 13;  // debug pin for overflow led 
int pin_mc0_led = 5;  // debug pin for compare led 
unsigned int loop_count = 0;
unsigned int irq_ovf_count = 0;


void blinkLED()
{
  static bool ledState = false;
  digitalWrite(HW_LED_PIN, ledState);
  ledState = !ledState;
}

// These are the two I2C functions in this sketch.
byte IIC_Read(byte regAddr)
{
  // This function reads one byte over IIC
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(regAddr);  // Address of CTRL_REG1
  Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
  Wire.requestFrom(MPL3115A2_ADDRESS, 1); // Request the data...
  return Wire.read();
}

void IIC_Write(byte regAddr, byte value)
{
  // This function writes one byto over IIC
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(regAddr);
  Wire.write(value);
  Wire.endTransmission(true);
}

void TC3_Handler()
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  if (TC->INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt
    digitalWrite(pin_ovf_led, irq_ovf_count % 2); // for debug leds
    digitalWrite(pin_mc0_led, HIGH); // for debug leds
    TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
    irq_ovf_count++;                 // for debug leds
  }
  
  if (TC->INTFLAG.bit.MC0 == 1) {  // A compare to cc0 caused the interrupt
    digitalWrite(pin_mc0_led, LOW);  // for debug leds
    TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
  
    //while (!imu.dataReady()); //Wait until new IMU data is available
    imu.update(UPDATE_ACCEL | UPDATE_GYRO);
    blinkLED();
    // If enabled, read from the compass.
    if (COMPASS_ENABLED)
    {
      imu.updateCompass();
    }
    //if (COMPASS_ENABLED && imu.updateCompass() != INV_SUCCESS)
    //  return; // If compass read fails (uh, oh) return to top
  
    //psensor.takeNextStep();
    
    //String imuLog = ""; //String(imu.time) + "\n"; // Create a fresh line to log
    //BoardData object for binary data logging
    BoardData bd;
    bd.pressure = 0; //Default to having no pressure
    /*if (!readingPressure) //psensor.hasNewData)
    {
  //    bd.pressure = psensor.pressure;
      //psensor.hasNewData = false;
      if(IIC_Read(MPL_STATUS) & (1<<2) == 0)
      {
        byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
        tempSetting &= ~(1<<1); //Clear OST bit
        IIC_Write(CTRL_REG1, tempSetting);
      
        tempSetting = IIC_Read(CTRL_REG1); //Read current settings to be safe
        tempSetting |= (1<<1); //Set OST bit
        IIC_Write(CTRL_REG1, tempSetting);
        readingPressure = true;
      }
    }
    else if (readingPressure)
    {
      if (IIC_Read(MPL_STATUS) & (1<<2))
      {
        //Time to read in pressure value
        Wire.beginTransmission(MPL3115A2_ADDRESS);
        Wire.write(OUT_P_MSB);  // Address of data to get
        Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
        byte msb, csb, lsb;
        msb = Wire.read();
        csb = Wire.read();
        lsb = Wire.read();

        byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
        tempSetting &= ~(1<<1); //Clear OST bit
        IIC_Write(CTRL_REG1, tempSetting);
      
        tempSetting = IIC_Read(CTRL_REG1); //Read current settings to be safe
        tempSetting |= (1<<1); //Set OST bit
        IIC_Write(CTRL_REG1, tempSetting);
        
        bd.pressure = (long)msb<<16 | (long)csb<<8 | (long)lsb;
      }
    }*/
  
    //Populate BoardData object
    bd.time = millis();
    bd.accel = (SensorData){.X = imu.ax, .Y = imu.ay, .Z = imu.az};
    bd.gyro = (SensorData){.X = imu.gx, .Y = imu.gy, .Z = imu.gz};
    bd.mag = (SensorData){.X = imu.mx, .Y = imu.my, .Z = imu.mz};
  
    databuffer.write(&bd, 1);
  }
}

void setup()
{

  /*delay(1000); //XXX: debug. Makes sure things can be reprogrammed
  blinkLED();
  delay(250);*/
  
  // Initialize LED, interrupt input, and serial port.
  // LED defaults to off:
  initHardware(); 

  /*while (1)
  {
    blinkLED();
    LOG_PORT.println("In the loop");
    delay(750);
  }*/

  // Initialize the MPU-9250. Should return true on success:
  if ( !initIMU() ) 
  {
    LOG_PORT.println("Error connecting to MPU-9250");
    while (1)
    {
      LOG_PORT.println("IMU failed to respond. This board is probably toast");
      blinkLED();
      delay(200);
      // Loop forever if we fail to connect
    }
    // LED will remain off in this state.
  }

  // Check for the presence of an SD card, and initialize it:
  if (initSD())
  {
    sdCardPresent = true;
    // Get the next, available log file name
    logFileName = nextLogFile(); 
  }

  // Open the current file name:
   logFile = SD.open(logFileName, FILE_WRITE);


     pinMode(pin_ovf_led, OUTPUT);   // for debug leds
  digitalWrite(pin_ovf_led, LOW); // for debug leds
  pinMode(pin_mc0_led, OUTPUT);   // for debug leds
  digitalWrite(pin_mc0_led, LOW); // for debug leds
  SerialUSB.begin(9600);

  
  // Enable clock for TC 
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID ( GCM_TCC2_TC3 ) ) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync 

  // The type cast must fit with the selected timer mode 
  TcCount16* TC = (TcCount16*) TC3; // get timer struct

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 

  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_NFRQ; // Set TC as normal Normal Frq
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 

  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;   // Set perscaler
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
  
  // TC->PER.reg = 0xFF;   // Set counter Top using the PER register but the 16/32 bit timer counts allway to max  
  // while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 

  TC->CC[0].reg = 0xFFF;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
  
  // Interrupts 
  TC->INTENSET.reg = 0;              // disable all interrupts
  TC->INTENSET.bit.OVF = 1;          // enable overfollow
  TC->INTENSET.bit.MC0 = 1;          // enable compare match to CC0

  // Enable InterruptVector
  NVIC_EnableIRQ(TC3_IRQn);

  // Enable TC
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
   /*while (1)
   {
    //blinkLED();
    LOG_PORT.println("In the setup end loop!");
    delay(100);
   }*/
}

void loop()
{
  while (millis() < last_loop_start + LOOP_TIME); //Pace execution time to desired sample rate
  last_loop_start = millis();
  
  // The loop constantly checks for new serial input:
  /*if (LOG_PORT.available())
  {
    // If new input is available on serial port
    parseSerialInput(LOG_PORT.read()); // parse it
  }*/

  
  // If logging (to either UART and SD card) is enabled
  //if ( enableSerialLogging || enableSDLogging)
  logIMUData(); // Log new data
}

void logIMUData(void)
{
  
  //if (enableSerialLogging)  // If serial port logging is enabled
  //LOG_PORT.write((uint8_t*)(&bd), sizeof(bd)); // Print log line to serial port

  // If SD card logging is enabled & a card is plugged in
  
  if (sdCardPresent)
  {
    if (databuffer.length() > 0)
    {
      
      //LOG_PORT.println("Data in the buffer");
      BoardData temp[30];
      uint16_t num_read = min(databuffer.length(), 30);
      databuffer.read(temp, num_read);
      databuffer.delete_oldest(num_read);
      sdLogBoardData(temp, num_read); // Log SD buffer
      //logFileBuffer = ""; // Clear SD log buffer 
      blinkLED(); // Blink LED every time a new buffer is logged to SD
    }
    // Add new line to SD log buffer
    //logFileBuffer += imuLog;
  }
  else
  {
    // Blink LED once every second (if only logging to serial port)
    if ( millis() > lastBlink + UART_BLINK_RATE )
    {
      blinkLED(); 
      lastBlink = millis();
    }
  }
}

void initHardware(void)
{
  // Set up LED pin (active-high, default to off)
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);

  // Set up MPU-9250 interrupt input (active-low)
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);

  // Set up serial log port
  LOG_PORT.begin(SERIAL_BAUD_RATE);
}

bool initIMU(void)
{
  // imu.begin() should return 0 on success. Will initialize
  // I2C bus, and reset MPU-9250 to defaults.
  if (imu.begin() != INV_SUCCESS)
    return false;

  // Set up MPU-9250 interrupt:
  imu.enableInterrupt(); // Enable interrupt output
  imu.setIntLevel(1);    // Set interrupt to active-low
  imu.setIntLatched(1);  // Latch interrupt output

  // Configure sensors:
  // Set gyro full-scale range: options are 250, 500, 1000, or 2000:
  imu.setGyroFSR(gyroFSR);
  // Set accel full-scale range: options are 2, 4, 8, or 16 g 
  imu.setAccelFSR(accelFSR);
  // Set gyro/accel LPF: options are5, 10, 20, 42, 98, 188 Hz
  imu.setLPF(IMU_AG_LPF); 
  // Set gyro/accel sample rate: must be between 4-1000Hz
  // (note: this value will be overridden by the DMP sample rate)
  imu.setSampleRate(IMU_AG_SAMPLE_RATE); 
  // Set compass sample rate: between 4-100Hz
  if (COMPASS_ENABLED)
    imu.setCompassSampleRate(IMU_COMPASS_SAMPLE_RATE); 

  // Configure digital motion processor. Use the FIFO to get
  // data from the DMP.
  unsigned short dmpFeatureMask = 0;
  if (ENABLE_GYRO_CALIBRATION)
  {
    // Gyro calibration re-calibrates the gyro after a set amount
    // of no motion detected
    dmpFeatureMask |= DMP_FEATURE_SEND_CAL_GYRO;
  }
  else
  {
    // Otherwise add raw gyro readings to the DMP
    dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
  }
  // Add accel and quaternion's to the DMP
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
  dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;

  // Initialize the DMP, and set the FIFO's update rate:
  imu.dmpBegin(dmpFeatureMask, fifoRate);

  return true; // Return success
}

bool initSD(void)
{
  // SD.begin should return true if a valid SD card is present
  if ( !SD.begin(SD_CHIP_SELECT_PIN) )
  {
    return false;
  }

  return true;
}

bool sdLogBoardData(BoardData* bd, uint16_t count)
{
  
  
  // If the file will get too big with this new string, create
  // a new one, and open it.
  if (logFile.size() > (SD_MAX_FILE_SIZE - sizeof(*bd)))
  {
    logFile.close();
    logFileName = nextLogFile();
    logFile = SD.open(logFileName, FILE_WRITE);
  }

  // If the log file opened properly, add the string to it.
  if (logFile)
  {
    //https://stackoverflow.com/a/5055648
    logFile.write(static_cast<uint8_t*>(static_cast<void*>(bd)), sizeof(*bd) * count);
    //logFile.close();
    if (last_loop_start - 1000 > lastlooplogged)
    {
      lastlooplogged = last_loop_start;
      logFile.flush();
    }
    return true; // Return success
  }

  return false; // Return fail
}

// Find the next available log file. Or return a null string
// if we've reached the maximum file limit.
String nextLogFile(void)
{
  String filename;
  int logIndex = 0;

  for (int i = 0; i < LOG_FILE_INDEX_MAX; i++)
  {
    // Construct a file with PREFIX[Index].SUFFIX
    filename = String(LOG_FILE_PREFIX);
    filename += String(logIndex);
    filename += ".";
    filename += String(LOG_FILE_SUFFIX);
    // If the file name doesn't exist, return it
    if (!SD.exists(filename))
    {
      return filename;
    }
    // Otherwise increment the index, and try again
    logIndex++;
  }

  return "";
}

// Parse serial input, take action if it's a valid character
void parseSerialInput(char c)
{
  unsigned short temp;
}
