/*
  T/LT Copyright 2014
*/

#include <boards.h>
#include <SPI.h>
#include <Wire.h>
#include "BLEFirmata.h"
#include <ble_shield.h>
#include <services.h>
#include "Thread.h"

/*==============================================================================
 * FEATURE DEFINES
 *============================================================================*/
/* DON'T TEST WITH THESE FEATURES NOW */
//#define DEBUG
//#define FREEFALL_MOTION_ENABLE
//#define BLE_FIRMATA
//#define TEST_BT_WITHOUT_ACCELEROMETER
//#define TEST_ACCELEROMETER_WITHOUT_BT

/*==============================================================================
 * ENUMERATIONS
 *============================================================================*/
typedef enum ble_cmd_tilt_to_phone_e {
  BLE_CMD_TILT_TO_PHONE_GENERAL_MSG = 0x0,
  BLE_CMD_TILE_TO_PHONE_BIKE_THEFT_NOTF_MSG = 0x1,
  BLE_CMD_TILE_TO_PHONE_MAX
} ble_cmd_tilt_to_phone_e_type;

typedef enum ble_cmd_phone_to_tilt_e {
  BLE_CMD_PHONE_TO_TILT_LIGHT_ENABLE_DISABLE = 0x0,
  BLE_CMD_PHONE_TO_TILT_SOUND_ENABLE_DISABLE,
  BLE_CMD_PHONE_TO_TILT_RESET,
  BLE_CMD_PHONE_TO_TILT_LIGHT_SOUND_ENABLE_DISABLE
} ble_cmd_phone_to_tilt_e_type;


/*==============================================================================
 * MACROS
 *============================================================================*/
 
// Bit-wise macros
//#define SETBIT(x,y)   (x |= (y))  //Set bit y in byte x 
//#define CLEARBIT(x,y) (x &= (~y)) //Clear bit Y in byte x 
#define CHECKBIT(x,y) ((y >> x) & 0x1)   //True if bit y of byte x=1. 

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

char *welcomeString = "Welcome to T/LT!";
char *bikeTheftNotfString = "Your bike is being moved!";
#define  TRUE  0x01
#define  FALSE  0x00

/*==============================================================================
 * I2C
 *============================================================================*/
 
// I2C Constants 
#define I2C_WRITE B00000000
#define I2C_READ B00001000
#define I2C_READ_CONTINUOUSLY B00010000
#define I2C_STOP_READING B00011000
#define I2C_READ_WRITE_MODE_MASK B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000
#define MAX_QUERIES 8
#define MINIMUM_SAMPLING_INTERVAL 10
#define REGISTER_NOT_SPECIFIED -1

/* I2C data */
struct i2c_device_info {
  byte addr;
  byte reg;
  byte bytes;
};

/* For I2C read continuous more */
i2c_device_info query[MAX_QUERIES];

byte i2cRxData[32];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
unsigned int i2cReadDelayTime = 0;  // default delay time between i2c read request and Wire.requestFrom()

/*==============================================================================
 * I2C
 *============================================================================*/

/* Analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/*==============================================================================
 * LIGHT/SOUND CONSTANTS
 *============================================================================*/
 
#define PWM_PIN            6
#define PWM_LEVEL_HIGH     127
#define PWM_LEVEL_MED      90
#define PWM_LEVEL_LOW      0
#define NOTE_B0            31
#define NOTE_FS7           2960
#define NOTE_DS5           622
#define LIGHT_FRONT        0
#define LIGHT_RIGHT        1
#define LIGHT_LEFT         2
#define LIGHT_BACK         3
#define SPEAKER_OUT        6
#define TYPE_OF_SOUND_FIND_BIKE  0
#define TYPE_OF_SOUND_ALARM      1

//sound configuration
const unsigned long maxAlarmTimeout = 5000;  // 5 seconds
unsigned long alarmStartTime = 0;
volatile boolean alarm_state = false;

/* Digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent
/* Pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written
/* Timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
int samplingInterval = 38;          // how often to run the main loop (in ms)
boolean showLight = false;
volatile boolean prevLightVal = false;
volatile boolean prevFindLight = false;
volatile unsigned int prevSoundVal = PWM_LEVEL_LOW;
boolean playSound = false;
static bool ledStatus = false;

/*==============================================================================
 * Threads()
 *============================================================================*/
Thread ledThread;

/*==============================================================================
 * ACCELEROMETER GLOBAL VARIABLES + CONSTANTS 
 *============================================================================*/
// The SparkFun breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
#define MMA8452_ADDRESS 0x1D  // 0x1D if SA0 is high, 0x1C if low

// Define a few of the registers that we will be accessing on the MMA8452
#define OUT_X_MSB 0x01
#define XYZ_DATA_CFG  0x0E
#define WHO_AM_I   0x0D

// Control registers
#define CTRL_REG1  0x2A
#define CTRL_REG2  0x2B
#define CTRL_REG3  0x2C
#define CTRL_REG4  0x2D
#define CTRL_REG5  0x2E
#define ASLP_COUNT  0x29

// Iniitial values of Control Registers
#define ASLP_12_5_HZ  0x40
#define ODR_50_HZ  0x20 
#define LNOISE_DEFAULT  0x00
#define F_READ_DEFAULT  0x00
#define SLPE  0x04  
#define SMODS_LOW_POW  0x18
#define MODS_NORMAL_POW  0x00
#define ST_DEFAULT  0x00
#define RST_DEFAULT  0x00
#define WAKE_FF_MT  0x08
#define WAKE_TRANS  0x40  
#define IPOL_DEFAULT 0x00
#define IPOL_ACTIVE_HIGH 0x02
#define PP_OD_DEFAULT 0x00
#define INT_EN_FF_MT  0x04
#define INT_EN_TRANS  0x20
#define INT_EN_DRDY  0x1
#define INT_EN_ASLP  0x80
#define INT_CFG_FF_MT  0x04
#define INT_CFG_TRANS  0x20
#define ASLP_COUNT_50_HZ_960_MS  0x03
#define ELE  0x80
#define OAE_MOTION  0x40
#define XYZ_EFE_FF 0x18
#define DBCNTM_MOTION  0x00
#define THS_0_2_G  0x0B
#define FF_MT_DEBOUNCE_50_HZ_20_MS  0x0A
#define ELE  0x10
#define XYZ_EFE_TRANS 0x0E
#define HPF_BYP 0x00
#define DBCNTM_TRANS  0x00 
// A working combination is observed at threshold = 4 and count = 5
#define TRANS_THS_0_5_G  0x03
#define TRANS_DEBOUNCE_50_HZ_20_MS  0x3
//The SRC registers for clearing and reading status of interrupts
#define SYSMOD 0x0B
#define TRANSIENT_SRC 0x1E     
#define FF_MT_SRC 0x16  
// Motion Detection registers
#define FF_MT_CFG  0x15
#define FF_MT_SRC  0x16
#define FF_MT_THS  0x17
#define FF_MT_COUNT  0x18
//Transient registers
#define TRANSIENT_CFG 0x1D
#define TRANSIENT_THS 0x1F
#define TRANSIENT_COUNT 0x20
#define INT_SOURCE  0x0C
#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.

/* Interrupt Status */
volatile unsigned int int_status = 0;
float threshX = 0.5;
float threshY = 0.3;
float threshZ = 0.5;
boolean accelConfgd = false;
boolean deviceSecurityEnabled = true;
int accelCount[3];  // Stores the 12-bit signed value
float initAccelG[3];
// Now we'll calculate the accleration value into actual g's
float accelG[3];  // Stores the real accel value in g's
int baudRate = 9600;
int regValTrans = 0;
int regValInt = 0;
int ztran = 0;
int ztran_pol = 0;
int ytran = 0;
int ytran_pol = 0;
int xtran = 0;
int xtran_pol = 0;
int ztran_bit = 5;
int ztran_pol_bit = 4;
int ytran_bit = 3;
int ytran_pol_bit = 2;
int xtran_bit = 1;
int xtran_pol_bit = 0;   

int zTranCount = 0;
int maxzTranCount = 0;
float xTranCount = 0;
float xTranZeroCount = 0;
float xTranOneCount = 0;
float minxThresh = 0.4;
float maxxThresh = 0.6;
int zTimerVal = 250;  // 0.5 second or 500000 us
int xTimerVal = 500;  // 3 seconds or 3000000 us
int zTimerStartTime = 0;
int xTimerStartTime = 0;
boolean zTimerStarted = false;
boolean xTimerStarted = false;
boolean triggerAlarm = false;
boolean alarmTriggered = false;



/*==============================================================================
 * FUNCTIONS
 *============================================================================*/

void readAndReportData(byte address, int theRegister, byte numBytes) {
  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()  
  if (theRegister != REGISTER_NOT_SPECIFIED) {
    Wire.beginTransmission(address);
    #if ARDUINO >= 100
    Wire.write((byte)theRegister);
    #else
    Wire.send((byte)theRegister);
    #endif
    Wire.endTransmission();
    delayMicroseconds(i2cReadDelayTime);  // delay is necessary for some devices such as WiiNunchuck
  } else {
    theRegister = 0;  // fill the register with a dummy value
  }

  Wire.requestFrom(address, numBytes);  // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if(numBytes == Wire.available()) {
    i2cRxData[0] = address;
    i2cRxData[1] = theRegister;
    for (int i = 0; i < numBytes; i++) {
      #if ARDUINO >= 100
      i2cRxData[2 + i] = Wire.read();
      #else
      i2cRxData[2 + i] = Wire.receive();
      #endif
    }
  }
  else {
    if(numBytes > Wire.available()) {
      BleFirmata.sendString("I2C Read Error: Too many bytes received");
    } else {
      BleFirmata.sendString("I2C Read Error: Too few bytes received"); 
    }
  }

  // send slave address, register and received bytes
  BleFirmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  
  // only send if the value is different than previously sent
  if(forceSend || previousPINs[portNumber] != portValue) {
    BleFirmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

/* -----------------------------------------------------------------------------
 * check all the active digital inputs for change of state, then add any events
 * to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */  
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
 * two bit-arrays that track Digital I/O and PWM status
 */
void setPinModeCallback(byte pin, int mode)
{
  if (pinConfig[pin] == I2C && isI2CEnabled && mode != I2C) {
    // disable i2c so pins can be used for other functions
    // the following if statements should reconfigure the pins properly
    disableI2CPins();
  }
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT) {
      portConfigInputs[pin/8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin/8] &= ~(1 << (pin & 7));
    }
  }
  pinState[pin] = 0;
  switch(mode) {
  case ANALOG:
    if (IS_PIN_ANALOG(pin)) {
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
      }
      pinConfig[pin] = ANALOG;
    }
    break;
  case INPUT:
    if (IS_PIN_DIGITAL(pin)) {      
      pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
      
      // Select your internal pull-up here
      digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
      //digitalWrite(PIN_TO_DIGITAL(pin), HIGH); // enable internal pull-ups if you have only a wire to test
      pinConfig[pin] = INPUT;
      
      // hack it only
      reportPINs[pin/8] |= (1 << (pin & 7));
    }
    break;
  case OUTPUT:
    if (IS_PIN_DIGITAL(pin)) {
      digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
      pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
      pinConfig[pin] = OUTPUT;
    }
    break;
  case PWM:
    if (IS_PIN_PWM(pin)) {
      pinMode(PIN_TO_PWM(pin), OUTPUT);
      analogWrite(PIN_TO_PWM(pin), 0);
      pinConfig[pin] = PWM;
    }
    break;
  case I2C:
    if (IS_PIN_I2C(pin)) {
      // mark the pin as i2c
      // the user must call I2C_CONFIG to enable I2C for a device
      pinConfig[pin] = I2C;
    }
    break;
  default:
    BleFirmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch(pinConfig[pin]) {
    case PWM:
      if (IS_PIN_PWM(pin))
        analogWrite(PIN_TO_PWM(pin), value);
        pinState[pin] = value;
      break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, mask=1, pinWriteMask=0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port*8+8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin=port*8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // only write to OUTPUT and INPUT (enables pullup)
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (pinConfig[pin] == OUTPUT || pinConfig[pin] == INPUT) {
          pinWriteMask |= mask;
          pinState[pin] = ((byte)value & mask) ? 1 : 0;
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}

// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
 */
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if(value == 0) {
      analogInputsToReport = analogInputsToReport &~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte slaveAddress;
  byte slaveRegister;
  byte data;
  unsigned int delayTime; 
  
  switch(command) {
  case I2C_REQUEST:
    mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
    if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
      BleFirmata.sendString("10-bit addressing mode is not yet supported");
      return;
    }
    else {
      slaveAddress = argv[0];
    }

    switch(mode) {
    case I2C_WRITE:
      Wire.beginTransmission(slaveAddress);
      for (byte i = 2; i < argc; i += 2) {
        data = argv[i] + (argv[i + 1] << 7);
        #if ARDUINO >= 100
        Wire.write(data);
        #else
        Wire.send(data);
        #endif
      }
      Wire.endTransmission();
      delayMicroseconds(70);
      break;
    case I2C_READ:
      if (argc == 6) {
        // a slave register is specified
        slaveRegister = argv[2] + (argv[3] << 7);
        data = argv[4] + (argv[5] << 7);  // bytes to read
        readAndReportData(slaveAddress, (int)slaveRegister, data);
      }
      else {
        // a slave register is NOT specified
        data = argv[2] + (argv[3] << 7);  // bytes to read
        readAndReportData(slaveAddress, (int)REGISTER_NOT_SPECIFIED, data);
      }
      break;
    case I2C_READ_CONTINUOUSLY:
      if ((queryIndex + 1) >= MAX_QUERIES) {
        // too many queries, just ignore
        BleFirmata.sendString("too many queries");
        break;
      }
      queryIndex++;
      query[queryIndex].addr = slaveAddress;
      query[queryIndex].reg = argv[2] + (argv[3] << 7);
      query[queryIndex].bytes = argv[4] + (argv[5] << 7);
      break;
    case I2C_STOP_READING:
	  byte queryIndexToSkip;      
      // if read continuous mode is enabled for only 1 i2c device, disable
      // read continuous reporting for that device
      if (queryIndex <= 0) {
        queryIndex = -1;        
      } else {
        // if read continuous mode is enabled for multiple devices,
        // determine which device to stop reading and remove it's data from
        // the array, shifiting other array data to fill the space
        for (byte i = 0; i < queryIndex + 1; i++) {
          if (query[i].addr = slaveAddress) {
            queryIndexToSkip = i;
            break;
          }
        }
        
        for (byte i = queryIndexToSkip; i<queryIndex + 1; i++) {
          if (i < MAX_QUERIES) {
            query[i].addr = query[i+1].addr;
            query[i].reg = query[i+1].addr;
            query[i].bytes = query[i+1].bytes; 
          }
        }
        queryIndex--;
      }
      break;
    default:
      break;
    }
    break;
  case I2C_CONFIG:
    delayTime = (argv[0] + (argv[1] << 7));

    if(delayTime > 0) {
      i2cReadDelayTime = delayTime;
    }

    if (!isI2CEnabled) {
      enableI2CPins();
    }
    
    break;
  case SAMPLING_INTERVAL:
    if (argc > 1) {
      samplingInterval = argv[0] + (argv[1] << 7);
      if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
        samplingInterval = MINIMUM_SAMPLING_INTERVAL;
      }      
    } else {
      //Firmata.sendString("Not enough data");
    }
    break;
  case EXTENDED_ANALOG:
    if (argc > 1) {
      int val = argv[1];
      if (argc > 2) val |= (argv[2] << 7);
      if (argc > 3) val |= (argv[3] << 14);
      analogWriteCallback(argv[0], val);
    }
    break;
  case CAPABILITY_QUERY:
    ble_write(START_SYSEX);
    ble_write(CAPABILITY_RESPONSE);
    for (byte pin=0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_DIGITAL(pin)) {
        ble_write((byte)INPUT);
        ble_write(1);
        ble_write((byte)OUTPUT);
        ble_write(1);
      }
      if (IS_PIN_ANALOG(pin)) {
        ble_write(ANALOG);
        ble_write(10);
      }
      if (IS_PIN_PWM(pin)) {
        ble_write(PWM);
        ble_write(8);
      }
      if (IS_PIN_SERVO(pin)) {
        ble_write(SERVO);
        ble_write(14);
      }
      if (IS_PIN_I2C(pin)) {
        ble_write(I2C);
        ble_write(1);  // to do: determine appropriate value 
      }
      ble_write(127);
    }
    ble_write(END_SYSEX);
    break;
  case PIN_STATE_QUERY:
    if (argc > 0) {
      byte pin=argv[0];
      ble_write(START_SYSEX);
      ble_write(PIN_STATE_RESPONSE);
      ble_write(pin);
      if (pin < TOTAL_PINS) {
        ble_write((byte)pinConfig[pin]);
	ble_write((byte)pinState[pin] & 0x7F);
	if (pinState[pin] & 0xFF80) ble_write((byte)(pinState[pin] >> 7) & 0x7F);
	if (pinState[pin] & 0xC000) ble_write((byte)(pinState[pin] >> 14) & 0x7F);
      }
      ble_write(END_SYSEX);
    }
    break;
  case ANALOG_MAPPING_QUERY:
    ble_write(START_SYSEX);
    ble_write(ANALOG_MAPPING_RESPONSE);
    for (byte pin=0; pin < TOTAL_PINS; pin++) {
      ble_write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
    }
    ble_write(END_SYSEX);
    break;
  }
}

void enableI2CPins()
{
  byte i;
  // is there a faster way to do this? would probaby require importing 
  // Arduino.h to get SCL and SDA pins
  for (i=0; i < TOTAL_PINS; i++) {
    if(IS_PIN_I2C(i)) {
      // mark pins as i2c so they are ignore in non i2c data requests
      setPinModeCallback(i, I2C);
    } 
  }
   
  isI2CEnabled = true; 
  
  // is there enough time before the first I2C request to call this here?
  Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins() {
    isI2CEnabled = false;
    // disable read continuous mode for all devices
    queryIndex = -1;
    // uncomment the following if or when the end() method is added to Wire library
    // Wire.end();
}

// Callback for myThread
void ledCallback() {
  ledStatus = !ledStatus;
}

/*==============================================================================
 * ISRs()
 *============================================================================*/
 
void int0_bh()
{
  regValInt = readRegister(INT_SOURCE);
  regValTrans = readRegister(TRANSIENT_SRC);
  // Read and print accel data
  readAndSaveAccelValues();
  int_status &= ~INT_EN_TRANS;
}

void int1_bh()
{
  int_status &= ~INT_EN_ASLP;
}

void systemResetCallback()
{
  // initialize a defalt state
  // TODO: option to load config from EEPROM instead of default
  if (isI2CEnabled) {
  	disableI2CPins();
  }
  for (byte i=0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;      // by default, reporting off
    portConfigInputs[i] = 0;	// until activated
    previousPINs[i] = 0;
  }
  // pins with analog capability default to analog input
  // otherwise, pins default to digital output
  for (byte i=0; i < TOTAL_PINS; i++) {

    // skip pin 8, 9 for BLE Shield
    if ((i == 8) || (i == 9))
      continue;
    
    // skip SPI pins
    if ( (i==MOSI) || (i==MISO) || (i==SCK) || (i==SS) )
      continue;
     
    // Default all to digital pins  
    setPinModeCallback(i, OUTPUT);
  }
  
  // By default, do not report any analog inputs
  analogInputsToReport = 0;
}


/*==============================================================================
 * SETUP()
 *============================================================================*/
void setup() 
{
  systemResetCallback();  // reset to default config

  // Enable serial debug
  Serial.begin(baudRate);

   // Init. BLE and start BLE library.
  ble_begin();

  pinMode(LIGHT_FRONT, OUTPUT);
  pinMode(LIGHT_LEFT, OUTPUT);
  pinMode(LIGHT_RIGHT, OUTPUT);
  pinMode(LIGHT_BACK, OUTPUT);
  pinMode(SPEAKER_OUT, OUTPUT);  
  resetPins();
  
  // Set LED blink timer for a value of 250 ms if enabled from phone
  noInterrupts();
//set timer1 interrupt at 4Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 3096;// = (16*10^6) / (4*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);  
  interrupts();
  Wire.begin(); //Join the bus as a master  

  initMMA8452(); //Test and intialize the MMA8452  
  //these interrupts causes the board to keep resetting
  
  //******************* THREAD LED ************************
  // Create a thread:
  ledThread = Thread();
  ledThread.onRun(ledCallback);
  ledThread.setInterval(1000);  
}

void readAndSaveAccelValues()
{
  memset(accelCount,0,sizeof(accelCount));
  readAccelData(accelCount);  // Read the x/y/z adc values 
  for (int i = 0 ; i < 3 ; i++) {
    accelG[i] = (float) accelCount[i] / ((1<<12)/(2*GSCALE));  // get actual g value, this depends on scale being set
  }  
}

void initAccelData()
{ 
  // Initialize current accel values
  readAndSaveAccelValues();
  initAccelG[0]=accelG[0];
  initAccelG[1]=accelG[1];
  initAccelG[2]=accelG[2];
  Serial.println("initAccelData(): Initial value recalculated:");
  Serial.println(initAccelG[0], 4);
  Serial.println(initAccelG[1], 4);
  Serial.println(initAccelG[2], 4); 
  setAccelConfigParm(true);
}

void setAccelConfigParm(boolean flag)
{
  Serial.print("setAccelConfigParm: Changing accel config flag to ");
  Serial.println(flag);
  accelConfgd = flag;
}

/* Master Theft Detection Logic */
/* Check delta between current accelData and X/Y/Z threshold values
   to decide whether someone is moving the bike or not. Return boolean
   TRUE if we think someone is moving the bike and FALSE otherwise. */
boolean monitorAccelData()
{
  if (!alarmTriggered && (millis() > maxAlarmTimeout))
  {
    if (zTimerStarted) {
      if ((millis() - zTimerStartTime) > zTimerVal) {
//        Serial.print("zTranCount = ");
//        Serial.println(zTranCount);
        if (zTranCount >= maxzTranCount) {
          triggerAlarm = true;   
          sendMsgtoPhone(BLE_CMD_TILE_TO_PHONE_BIKE_THEFT_NOTF_MSG);          
//          Serial.println("Z-axis alarm");        
        }
        zTimerStarted = false;
//        Serial.println("Z-timer stopped");
      }
    } else if (xTimerStarted) {
      if ((millis() - xTimerStartTime) > xTimerVal) {    
        float xPolRatio = xTranOneCount/xTranCount;
//        Serial.print("xPolRatio = ");
//        Serial.print(xPolRatio, 4);
//        Serial.print(", xTranOneCount = ");
//        Serial.print(xTranOneCount);
//        Serial.print(", xTranZeroCount = ");
//        Serial.print(xTranZeroCount);
//        Serial.print(", xTranCount = ");
//        Serial.println(xTranCount);
        if ((xPolRatio > maxxThresh) || (xPolRatio < minxThresh)) {
          triggerAlarm = true;
          
//          Serial.println("X-axis alarm");
        }   
        xTimerStarted = false;  
//        Serial.println("X-timer stopped");      
      }
    }
  }
  else
  {
    if (zTimerStarted) {
      zTimerStarted = false;
//      Serial.println("[monitorAccelData]: Ignoring Z trigger!!!");
    }
    if (xTimerStarted) {
      xTimerStarted = false;
//      Serial.println("[monitorAccelData]: Ignoring X trigger!!!");
    } 
  }
}

void resetPins()
{
  activate_sound_only(PWM_LEVEL_LOW, TYPE_OF_SOUND_FIND_BIKE);  
  activate_light_only(LOW);  
}

ISR(TIMER1_COMPA_vect) {
  // Generates pulse wave of frequency 4 Hz
//  if (alarm_state) {    
//    prevLightVal = !prevLightVal;
//    activate_light_only(prevLightVal);  
//  }
//  
//  if (showLight) {
//    prevLightVal = !prevLightVal;
//    activate_light_only(prevLightVal);  
//  }
  
  if (playSound) {
    prevSoundVal = (prevSoundVal == PWM_LEVEL_LOW) ? PWM_LEVEL_MED : PWM_LEVEL_LOW;
    activate_light_only(prevSoundVal);
  }  
}

void activate_light_only(boolean state){
  digitalWrite(LIGHT_FRONT, state);   // set the LED to state
  digitalWrite(LIGHT_RIGHT, state);   // set the LED to state
  digitalWrite(LIGHT_LEFT, state);   // set the LED to state
  digitalWrite(LIGHT_BACK, state);   // set the LED to state
  delay(100);              // wait a lil while. 1000 is one second
}

void activate_sound_only(int level, int type) {
  switch (type) {
    case TYPE_OF_SOUND_FIND_BIKE:
      analogWrite(PWM_PIN, level);    
      break;
    case TYPE_OF_SOUND_ALARM:
      digitalWrite(PWM_PIN, level);
      break;
    default:
      break; 
  }
}

void alarm_handler(boolean active) {  
  prevLightVal = active;
  
  if (alarm_state == false) {
    if (active == true) {    
      activate_sound_only(active, TYPE_OF_SOUND_ALARM);
      activate_light_only(active);
      alarm_state = true;   
      Serial.println("Alarm triggered!!");       
    } else {
      activate_light_only(active);
      activate_sound_only(active, TYPE_OF_SOUND_ALARM);    
      Serial.println("Alarm disabled!!");             
    }
  } 
}

void handlePlaySoundCmd(byte value)
{
  if (value == 0x01) {
    playSound = true;
    activate_sound_only(PWM_LEVEL_MED, TYPE_OF_SOUND_FIND_BIKE);  
  } else {
    playSound = false;
    activate_sound_only(PWM_LEVEL_LOW, TYPE_OF_SOUND_FIND_BIKE);
  }   
}

void handleShowLightCmd(byte value)
{
  if (value == 0x01) {
    showLight = true;
    //activate_light_only(HIGH);
  } else {
    showLight = false;
    //prevFindLight = LOW;
    //activate_light_only(prevLightVal);
  } 
}

void handleLightAndSound(byte value)
{
  handlePlaySoundCmd(value);
  handleShowLightCmd(value);
}

void sendMsgtoPhone(int cmd_id)
{ 
  if (cmd_id < BLE_CMD_TILE_TO_PHONE_MAX) {
    ble_write(cmd_id);
  }
}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void loop() 
{ 
  unsigned long mil = 0;

//********************* BLUETOOTH *******************
  // Byte 0: Command
  // Byte 1: Command Value

  while (ble_available()) {
    // Read out command and data
    byte data0 = ble_read();
    byte data1 = ble_read();
    byte data2 = ble_read();
    
    switch (data0) {
      case BLE_CMD_PHONE_TO_TILT_LIGHT_ENABLE_DISABLE:
        // Turn on/off light
        handleShowLightCmd(data1);
        break;
      
      case BLE_CMD_PHONE_TO_TILT_SOUND_ENABLE_DISABLE:
        // Turn on/off sound
        handlePlaySoundCmd(data1);        
        break;
      
      case BLE_CMD_PHONE_TO_TILT_RESET:
        // Reset PINs. Sent from device upon connection. Also send a welcome message.
        resetPins();      
        break;
      
      case BLE_CMD_PHONE_TO_TILT_LIGHT_SOUND_ENABLE_DISABLE:
        handleLightAndSound(data1);
        break;
        
      default:
        break;  
    }
  }

  //Allow BLE Shield to send/receive data
  ble_do_events();
//********************* BLUETOOTH *******************

  monitorAccelData();
  
  if (triggerAlarm) {
    alarmStartTime = millis(); 
    alarm_handler(true);
    alarm_state = true;
    triggerAlarm = false;
    alarmTriggered = true;
  }
  
  if (alarm_state == true) {
    if ((millis() - alarmStartTime) >= maxAlarmTimeout) {
      alarm_state = false;    
      alarm_handler(false);
      alarmTriggered = false;
      mil = millis();   
    }
  }

  // checks if thread should run (ie., if alarm triggered)
  if (ledThread.shouldRun()) {
    ledThread.run();
    delay(100);
    
    // Show the lights either if 1. Alarm is enabled 2. User is trying to find their bike
    if (alarmTriggered || showLight) {
      activate_light_only(ledStatus);
//      digitalWrite(ledPin, ledStatus);
//      Serial.print("LED status: ");
//      Serial.println(ledStatus);
//      Serial.print("LED pin: ");          
//      Serial.println(ledPin);       
    } else if (ledStatus)  {
      // ensure that LED is kept low when alarm is not triggered or when user is NOT trying to find their bik
      activate_light_only(LOW);
//      digitalWrite(ledPin, 0);
//      Serial.print("LED status: ");
//      Serial.println(ledStatus);
//      Serial.print("LED pin: ");          
//      Serial.println(ledPin);            
    }
  }
  
  //***************** Accelerometer **********************************//
  // Read INT SOURCE register
  int_status = readRegister(INT_SOURCE);
  
  // Detect transient interrupt
  if (int_status & INT_EN_TRANS) {
    int0_bh(); 
 
    ztran = CHECKBIT(ztran_bit, regValTrans);
    ztran_pol = CHECKBIT(ztran_pol_bit, regValTrans);
    ytran = CHECKBIT(ytran_bit, regValTrans);
    ytran_pol = CHECKBIT(ytran_pol_bit, regValTrans);
    xtran = CHECKBIT(xtran_bit, regValTrans);
    xtran_pol = CHECKBIT(xtran_pol_bit, regValTrans);  

    if (ztran) {
      if (!zTimerStarted) {
        zTimerStarted = true;
        zTranCount = 0;
        zTimerStartTime = millis();      
      } else {
        zTranCount++;
      }  
    }
    
    if (xtran_pol == 0x1) {
      if (!xTimerStarted) {
        xTimerStarted = true;
        xTranCount = 1;
        xTranZeroCount = 0;
        xTranOneCount = 1; 
        xTimerStartTime = millis();  
      } else {
        xTranOneCount++;
        xTranCount++;
      }
    } else {
      if (!xTimerStarted) {
        xTimerStarted = true;
        xTranCount = 1;  
        xTranZeroCount = 1;
        xTranOneCount = 0;   
        xTimerStartTime = millis();            
      } else {
        xTranZeroCount++;
        xTranCount++;
      }
    }    
  }

//***************** Accelerometer : END **********************************//
}

/*==============================================================================
 * ACCELEROMETER METHODS
 *============================================================================*/
void readAccelData(int *destination)
{
  byte rawData[6];  // x/y/z accel register data stored here
  readRegisters(OUT_X_MSB, 6, rawData);  // Read the six raw data registers into data array

  // Loop to calculate 12-bit ADC and g value for each axis
  for (int i = 0; i < 3 ; i++) {
    int gCount = (rawData[i*2] << 8) | rawData[(i*2)+1];  //Combine the two 8 bit registers into one 12-bit number
    gCount >>= 4; //The registers are left align, here we right align the 12-bit integer
    // If the number is negative, we have to make it so manually (no 12-bit data type)
    if (rawData[i*2] > 0x7F) {  
      gCount = ~gCount + 1;
      gCount *= -1;  // Transform into negative 2's complement #
    }

    destination[i] = gCount; //Record this gCount into the 3 int array
  }
}

// Initialize the MMA8452 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
void initMMA8452()
{
  byte val = readRegister(WHO_AM_I);  // Read WHO_AM_I register
  if (val == 0x2A) // WHO_AM_I should always be 0x2A
  {  
    Serial.println("M is online...");
  }
  else
  {
    Serial.print("Could not connect to MMA8452Q: 0x");
    Serial.println(val, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  MMA8452Standby();  // Must be in standby to change registers

  // Initialize GSCALE
  val = GSCALE;
  if (val > 8) val = 8; //Easy error check
  val >>= 2; // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
  writeRegister(XYZ_DATA_CFG, val);

  val = (ASLP_12_5_HZ | ODR_50_HZ | LNOISE_DEFAULT | F_READ_DEFAULT);
  writeRegister(CTRL_REG1, val);
  
  val = (ST_DEFAULT | RST_DEFAULT | SMODS_LOW_POW | SLPE | MODS_NORMAL_POW);
  writeRegister(CTRL_REG2, val);

  val = (WAKE_TRANS | IPOL_ACTIVE_HIGH | PP_OD_DEFAULT);
  writeRegister(CTRL_REG3, val);
  
  val = INT_CFG_TRANS;
  writeRegister(CTRL_REG5, val);  
  
  val = ASLP_COUNT_50_HZ_960_MS;
  writeRegister(ASLP_COUNT, val); 
  
#ifdef FREEFALL_MOTION_ENABLE 
  val = (ELE | OAE_MOTION | XYZ_EFE_FF);
  writeRegister(FF_MT_CFG, val);
  
  val = (DBCNTM_MOTION | THS_0_2_G);
  writeRegister(FF_MT_THS, val);  
  
  val = FF_MT_DEBOUNCE_50_HZ_20_MS;
  writeRegister(FF_MT_COUNT, val);  
#else
  val = (ELE | XYZ_EFE_TRANS | HPF_BYP);
  writeRegister(TRANSIENT_CFG, val);
  
  val = (DBCNTM_TRANS | TRANS_THS_0_5_G);
  writeRegister(TRANSIENT_THS, val);
  
  val = TRANS_DEBOUNCE_50_HZ_20_MS;
  writeRegister(TRANSIENT_COUNT, val);    
#endif /* FREEFALL_MOTION_ENABLE */
  
  val = (INT_EN_TRANS|INT_EN_ASLP);
  writeRegister(CTRL_REG4, val);

  MMA8452Active();  // Set to active to start reading  
}

// Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}

// Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}

// Read bytesToRead sequentially, starting at addressToRead into the dest byte array
void readRegisters(byte addressToRead, int bytesToRead, byte * dest)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active
  Wire.requestFrom(MMA8452_ADDRESS, bytesToRead); //Ask for bytes, once done, bus is released by default
  while (Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect

  for (int x = 0 ; x < bytesToRead ; x++) {
    dest[x] = Wire.read();  
  }
  
  Wire.endTransmission(true); //endTransmission but keep the connection active  
}

// Read a single byte from addressToRead and return it as a byte
byte readRegister(byte addressToRead)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active 
  Wire.requestFrom(MMA8452_ADDRESS, 1); //Ask for 1 byte, once done, bus is released by default
  while(!Wire.available()) ; //Wait for the data to come back 
  return Wire.read(); //Return this one byte
}

// Writes a single byte (dataToWrite) into addressToWrite
void writeRegister(byte addressToWrite, byte dataToWrite)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToWrite);
  Wire.write(dataToWrite);
  Wire.endTransmission(); //Stop transmitting
}
