
#import "BLE.h"
#import "BLEDefines.h"


@implementation BLE

@synthesize delegate;
@synthesize CM;
@synthesize peripherals;
@synthesize activePeripheral;
@synthesize services;
@synthesize serviceUUIDs;
@synthesize currentRSSI;

static BLE *bleSingleton;
static BOOL isConnected = false;

+ (BLE*)getInstance
{
  if (bleSingleton == nil) {
    bleSingleton = [[BLE alloc] init];
    [bleSingleton initializeData];
  }
  
  return bleSingleton;
}

-(void)initializeData
{
  // Initialize CBCentralManager object
  CM = [[CBCentralManager alloc] initWithDelegate:self queue:nil];
  
  // Initialize all monitoring UUID services
  services = [[NSMutableDictionary alloc] init];
  [services setObject:[CBUUID UUIDWithString:@"1811"] forKey:@"AlertNotificationService"];
  [services setObject:[CBUUID UUIDWithString:@"180F"] forKey:@"BatteryService"];
  [services setObject:[CBUUID UUIDWithString:@"1805"] forKey:@"CurrentTimeService"];
  [services setObject:[CBUUID UUIDWithString:@"180A"] forKey:@"DeviceInformation"];
  [services setObject:[CBUUID UUIDWithString:@"1804"] forKey:@"TxPower"];
  [services setObject:[CBUUID UUIDWithString:@"1803"] forKey:@"LinkLoss"];
  [services setObject:[CBUUID UUIDWithString:@"713D0000-503E-4C75-BA94-3148F18D941E"] forKey:@"RBLService"];
  [services setObject:[CBUUID UUIDWithString:@"713D0002-503E-4C75-BA94-3148F18D941E"] forKey:@"RBLTx"];
  [services setObject:[CBUUID UUIDWithString:@"713D0003-503E-4C75-BA94-3148F18D941E"] forKey:@"RBLRx"];
  
  // Initialize UUID array
  serviceUUIDs = [[NSMutableArray alloc] initWithCapacity:[services count]];
  for (id key in services) {
    [serviceUUIDs addObject:[services objectForKey:key]];
  }
}

- (void) readRSSI
{
  [activePeripheral readRSSI];
}

- (BOOL)isConnected
{
  return isConnected;
}

- (void)read
{
  CBUUID *uuid_service = [services objectForKey:@"RBLService"];
  CBUUID *uuid_char = [services objectForKey:@"RBLTx"];
  [self readValue:uuid_service characteristicUUID:uuid_char p:activePeripheral];
}

- (void)write:(NSData *)d
{
  CBUUID *uuid_service = [services objectForKey:@"RBLService"];
  CBUUID *uuid_char = [services objectForKey:@"RBLRx"];
  [self writeValue:uuid_service characteristicUUID:uuid_char p:activePeripheral data:d];
}

- (void) enableReadNotification:(CBPeripheral *)p
{
  CBUUID *uuid_service = [services objectForKey:@"RBLService"];
  CBUUID *uuid_char = [services objectForKey:@"RBLTx"];
  
  [self notification:uuid_service characteristicUUID:uuid_char p:p on:YES];
}

- (void)notification:(CBUUID *)serviceUUID characteristicUUID:(CBUUID *)characteristicUUID p:(CBPeripheral *)p on:(BOOL)on
{
  CBService *service = [self findServiceFromUUID:serviceUUID p:p];
  
  if (!service)
  {
    NSLog(@"notification: Could not find service with UUID %@ on peripheral with UUID %@",
           [self CBUUIDToString:serviceUUID],
           p.identifier.UUIDString);
    
    return;
  }
  
  CBCharacteristic *characteristic = [self findCharacteristicFromUUID:characteristicUUID service:service];
  
  if (!characteristic) {
    NSLog(@"notification: Could not find characteristic with UUID %@ on service with UUID %@ on peripheral with UUID %@",
          [self CBUUIDToString:characteristicUUID],
          [self CBUUIDToString:serviceUUID],
          p.identifier.UUIDString);
    
    return;
  }
  
  [p setNotifyValue:on forCharacteristic:characteristic];
}

- (UInt16)frameworkVersion
{
  return RBL_BLE_FRAMEWORK_VER;
}

- (NSString *)CBUUIDToString:(CBUUID *)cbuuid;
{
  NSData *data = cbuuid.data;
  
  if ([data length] == 2) {
    const unsigned char *tokenBytes = [data bytes];
    return [NSString stringWithFormat:@"%02x%02x", tokenBytes[0], tokenBytes[1]];
  }
  else if ([data length] == 16) {
    NSUUID* nsuuid = [[NSUUID alloc] initWithUUIDBytes:[data bytes]];
    return [nsuuid UUIDString];
  }
  
  return [cbuuid description];
}

- (void)readValue:(CBUUID *)serviceUUID characteristicUUID:(CBUUID *)characteristicUUID p:(CBPeripheral *)p
{
  CBService *service = [self findServiceFromUUID:serviceUUID p:p];
  
  if (!service) {
    NSLog(@"readValue: Could not find service with UUID %@ on peripheral with UUID %@",
          [self CBUUIDToString:serviceUUID],
          p.identifier.UUIDString);
    
    return;
  }
  
  CBCharacteristic *characteristic = [self findCharacteristicFromUUID:characteristicUUID service:service];
  
  if (!characteristic) {
    NSLog(@"readValue: Could not find characteristic with UUID %@ on service with UUID %@ on peripheral with UUID %@",
          [self CBUUIDToString:characteristicUUID],
          [self CBUUIDToString:serviceUUID],
          p.identifier.UUIDString);
    
    return;
  }
  
  [p readValueForCharacteristic:characteristic];
}

- (void)writeValue:(CBUUID *)serviceUUID characteristicUUID:(CBUUID *)characteristicUUID p:(CBPeripheral *)p data:(NSData *)data
{
  CBService *service = [self findServiceFromUUID:serviceUUID p:p];
  
  if (!service) {
    NSLog(@"writeValue: Could not find service with UUID %@ on peripheral with UUID %@",
          [self CBUUIDToString:serviceUUID],
          p.identifier.UUIDString);
    
    return;
  }
  
  CBCharacteristic *characteristic = [self findCharacteristicFromUUID:characteristicUUID service:service];
  
  if (!characteristic) {
    NSLog(@"writeValue: Could not find characteristic with UUID %@ on service with UUID %@ on peripheral with UUID %@",
          [self CBUUIDToString:characteristicUUID],
          [self CBUUIDToString:serviceUUID],
          p.identifier.UUIDString);
    
    return;
  }
  
  [p writeValue:data forCharacteristic:characteristic type:CBCharacteristicWriteWithoutResponse];
}

- (int)findBLEPeripherals:(int)timeout
{
  if (self.CM.state != CBCentralManagerStatePoweredOn) {
    NSLog(@"findBLEPeripherals: CoreBluetooth not correctly initialized!");
    NSLog(@"findBLEPeripherals: State = %ld (%s)\r\n", self.CM.state, [self centralManagerStateToString:self.CM.state]);
    return -1;
  }
  
  [NSTimer scheduledTimerWithTimeInterval:(float)timeout target:self selector:@selector(scanTimer:) userInfo:nil repeats:NO];
  [CM scanForPeripheralsWithServices:serviceUUIDs options:nil];

  NSLog(@"scanForPeripheralsWithServices");
  return 0; // Started scanning OK !
}

- (void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error;
{
  [delegate bleDidDisconnect];
  isConnected = false;
}

- (void)connectPeripheral:(CBPeripheral *)peripheral
{
  NSLog(@"connectPeripheral: Connecting to peripheral with UUID : %@", peripheral.identifier.UUIDString);
  
  activePeripheral = peripheral;
  activePeripheral.delegate = self;
  [CM connectPeripheral:activePeripheral
                     options:[NSDictionary dictionaryWithObject:[NSNumber numberWithBool:YES] forKey:CBConnectPeripheralOptionNotifyOnDisconnectionKey]];
}

- (void)scanTimer:(NSTimer *)timer
{
  [CM stopScan];
  NSLog(@"scanTimer: Stopped Scanning. Known peripherals : %lu", (unsigned long)[self.peripherals count]);
  [self printKnownPeripherals];
}

- (void)getAllServicesFromPeripheral:(CBPeripheral *)p
{
  [p discoverServices:serviceUUIDs];
}

- (void)getAllCharacteristicsFromPeripheral:(CBPeripheral *)p
{
  for (int i=0; i < p.services.count; i++) {
    CBService *s = [p.services objectAtIndex:i];
    [p discoverCharacteristics:nil forService:s];
  }
}

# pragma CoreBluetooth routine handlers

- (void)centralManagerDidUpdateState:(CBCentralManager *)central
{
  NSLog(@"centralManagerDidUpdateState: Status of CoreBluetooth central manager changed %ld (%s) - BT state %d", central.state, [self centralManagerStateToString:central.state], isConnected);
  
  // Try to autoconnect if we're not connected and the CB state just changed to CBCentralManagerStatePoweredOn
  if (!isConnected && central.state == CBCentralManagerStatePoweredOn) {
    [delegate bleDidChangedStateToPoweredOn];
  }
}

- (void)centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)peripheral advertisementData:(NSDictionary *)advertisementData RSSI:(NSNumber *)RSSI
{
  if (!peripherals) {
    peripherals = [[NSMutableArray alloc] initWithObjects:peripheral, nil];
  }
  else {
    for (int i = 0; i < peripherals.count; i++) {
      CBPeripheral *p = [peripherals objectAtIndex:i];
      
      if ((p.identifier == NULL) || (peripheral.identifier == NULL)) continue;
      
      if ([self UUIDSAreEqual:p.identifier UUID2:peripheral.identifier]) {
        [peripherals replaceObjectAtIndex:i withObject:peripheral];
        NSLog(@"centralManager:didDiscoverPeripheral: Duplicate UUID found updating...");
        return;
      }
    }
    
    [peripherals addObject:peripheral];
    NSLog(@"centralManager:centralManager:didDiscoverPeripheral: New UUID, adding");
  }
  
  NSLog(@"centralManager:didDiscoverPeripheral");
}

- (void)centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)peripheral
{
  if (peripheral.identifier != NULL) {
    NSLog(@"Connected to %@ successful", peripheral.identifier.UUIDString);
  } else {
    NSLog(@"Connected to NULL successful");
  }
  
  activePeripheral = peripheral;
  [activePeripheral discoverServices:nil];
  [self getAllServicesFromPeripheral:peripheral];
}

- (void)peripheral:(CBPeripheral *)peripheral didDiscoverCharacteristicsForService:(CBService *)service error:(NSError *)error
{
  if (!error) {
    for (int i=0; i < service.characteristics.count; i++) {
      CBService *s = [peripheral.services objectAtIndex:(peripheral.services.count - 1)];
      if ([service.UUID isEqual:s.UUID]) {
        if (!isConnected) {
          [self enableReadNotification:activePeripheral];
          [delegate bleDidConnect];
          isConnected = true;
        }
        break;
      }
    }
  } else {
    NSLog(@"peripheral:didDiscoverCharacteristicsForService Characteristic discorvery unsuccessful!");
  }
}

- (void)peripheral:(CBPeripheral *)peripheral didDiscoverServices:(NSError *)error
{
  if (!error) {
    [self getAllCharacteristicsFromPeripheral:peripheral];
  } else {
    NSLog(@"peripheral:didDiscoverServices: Service discovery was unsuccessful!");
  }
}

- (void)peripheral:(CBPeripheral *)peripheral didUpdateNotificationStateForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error
{
  if (!error) {
    //        printf("Updated notification state for characteristic with UUID %s on service with  UUID %s on peripheral with UUID %s\r\n",[self CBUUIDToString:characteristic.UUID],[self CBUUIDToString:characteristic.service.UUID],[self UUIDToString:peripheral.UUID]);
  } else {
    NSLog(@"peripheral:didUpdateNotificationStateForCharacteristic: Error in setting notification state for characteristic with UUID %@ on service with UUID %@ on peripheral with UUID %@",
          [self CBUUIDToString:characteristic.UUID],
          [self CBUUIDToString:characteristic.service.UUID],
          peripheral.identifier.UUIDString);
    
    NSLog(@"peripheral:didUpdateNotificationStateForCharacteristic: Error code was %s", [[error description] cStringUsingEncoding:NSStringEncodingConversionAllowLossy]);
  }
}

- (void)peripheral:(CBPeripheral *)peripheral didUpdateValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error
{
  unsigned char data[MAX_RX_BUF_PKT_SZ];
  static unsigned char buf[512];
  static int len = 0;
  NSInteger data_len;
  
  if (!error) {
    if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:@RBL_CHAR_TX_UUID]]) {
      data_len = characteristic.value.length;
      [characteristic.value getBytes:data length:data_len];
      
      if (data_len == MAX_RX_BUF_PKT_SZ) {
        memcpy(&buf[len], data, MAX_RX_BUF_PKT_SZ);
        len += data_len;
        
        if (len >= 64) {
          [delegate bleDidReceiveData:buf length:len];
          len = 0;
        }
      } else if (data_len < MAX_RX_BUF_PKT_SZ) {
        memcpy(&buf[len], data, data_len);
        len += data_len;
        
        [delegate bleDidReceiveData:buf length:len];
        len = 0;
      }
    }
  } else {
    NSLog(@"peripheral: updateValueForCharacteristic failed!");
  }
}

- (void)peripheralDidUpdateRSSI:(CBPeripheral *)peripheral error:(NSError *)error
{
  if (!isConnected) {
    NSLog(@"peripheralDidUpdateRSSI: Not Connected");
    return;
  }
  
  if (currentRSSI != peripheral.RSSI)
  {
    currentRSSI = peripheral.RSSI;
    [delegate bleDidUpdateRSSI:activePeripheral.RSSI];
  }
}

# pragma Utility functions

- (UInt16)swap:(UInt16)s
{
  UInt16 temp = s << 8;
  temp |= (s >> 8);
  return temp;
}

- (const char *)centralManagerStateToString:(int)state
{
  switch(state) {
    case CBCentralManagerStateUnknown:
      return "State unknown (CBCentralManagerStateUnknown)";
    case CBCentralManagerStateResetting:
      return "State resetting (CBCentralManagerStateUnknown)";
    case CBCentralManagerStateUnsupported:
      return "State BLE unsupported (CBCentralManagerStateResetting)";
    case CBCentralManagerStateUnauthorized:
      return "State unauthorized (CBCentralManagerStateUnauthorized)";
    case CBCentralManagerStatePoweredOff:
      return "State BLE powered off (CBCentralManagerStatePoweredOff)";
    case CBCentralManagerStatePoweredOn:
      return "State powered up and ready (CBCentralManagerStatePoweredOn)";
    default:
      return "State unknown";
  }
}

- (void)printKnownPeripherals
{
  NSLog(@"printKnownPeripherals: List of currently known peripherals :");
  
  for (int i = 0; i < self.peripherals.count; i++) {
    CBPeripheral *p = [self.peripherals objectAtIndex:i];
    
    if (p.identifier != NULL) {
      NSLog(@"%d  |  %@", i, p.identifier.UUIDString);
    }
    else {
      NSLog(@"%d  |  NULL", i);
    }
    
    [self printPeripheralInfo:p];
  }
}

- (void)printPeripheralInfo:(CBPeripheral*)peripheral
{
  NSLog(@"------------------------------------");
  NSLog(@"Peripheral Info :");
  
  if (peripheral.identifier != NULL) {
    NSLog(@"UUID : %@", peripheral.identifier.UUIDString);
  }
  else {
    NSLog(@"UUID : NULL");
  }
  
  NSLog(@"Name : %@", peripheral.name);
  NSLog(@"-------------------------------------");
}

- (BOOL)UUIDSAreEqual:(NSUUID *)UUID1 UUID2:(NSUUID *)UUID2
{
  return ([UUID1.UUIDString isEqualToString:UUID2.UUIDString]);
}

- (int)compareCBUUID:(CBUUID *) UUID1 UUID2:(CBUUID *)UUID2
{
  char b1[16];
  char b2[16];
  [UUID1.data getBytes:b1];
  [UUID2.data getBytes:b2];
  
  if (memcmp(b1, b2, UUID1.data.length) == 0) return 1;
  else return 0;
}

- (int)compareCBUUIDToInt:(CBUUID *)UUID1 UUID2:(UInt16)UUID2
{
  char b1[16];
  
  [UUID1.data getBytes:b1];
  UInt16 b2 = [self swap:UUID2];
  
  if (memcmp(b1, (char *)&b2, 2) == 0) return 1;
  else return 0;
}

- (UInt16)CBUUIDToInt:(CBUUID *) UUID
{
  char b1[16];
  [UUID.data getBytes:b1];
  return ((b1[0] << 8) | b1[1]);
}

- (CBUUID *)IntToCBUUID:(UInt16)UUID
{
  char t[16];
  t[0] = ((UUID >> 8) & 0xff); t[1] = (UUID & 0xff);
  NSData *data = [[NSData alloc] initWithBytes:t length:16];
  return [CBUUID UUIDWithData:data];
}

- (CBService *)findServiceFromUUID:(CBUUID *)UUID p:(CBPeripheral *)p
{
  for (int i = 0; i < p.services.count; i++) {
    CBService *s = [p.services objectAtIndex:i];
    if ([self compareCBUUID:s.UUID UUID2:UUID]) return s;
  }
  
  return nil; //Service not found on this peripheral
}

- (CBCharacteristic *)findCharacteristicFromUUID:(CBUUID *)UUID service:(CBService*)service
{
  for (int i = 0; i < service.characteristics.count; i++) {
    CBCharacteristic *c = [service.characteristics objectAtIndex:i];
    if ([self compareCBUUID:c.UUID UUID2:UUID]) return c;
  }
  
  return nil; //Characteristic not found on this service
}

@end
