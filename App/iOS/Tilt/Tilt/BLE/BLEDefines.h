// RBL Service
#define RBL_SERVICE_UUID                         "713D0000-503E-4C75-BA94-3148F18D941E"
#define RBL_CHAR_TX_UUID                         "713D0002-503E-4C75-BA94-3148F18D941E"
#define RBL_CHAR_RX_UUID                         "713D0003-503E-4C75-BA94-3148F18D941E"
#define RBL_BLE_FRAMEWORK_VER                    0x0200

typedef NS_ENUM(NSUInteger, BleCmdPhoneToTilt) {
  kTurnOnLight = 0x0,
  kPlaySound,
  kResetPins
} NS_AVAILABLE(NA, 7_0);

typedef NS_ENUM(NSUInteger, BleCmdTiltToPhone) {
  kGeneralMsg = 0x0
} NS_AVAILABLE(NA, 7_0);
