//
//  TiltViewController.m
//  Tilt
//
//  Created by Abhishek Sen on 12/27/13.
//  Copyright (c) 2013 Tilt. All rights reserved.
//

#import "TiltViewController.h"
#import "BLE.h"
#import "BLEDefines.h"

@interface TiltViewController ()

@property (weak, nonatomic) IBOutlet UIButton *btnConnect;
@property (weak, nonatomic) IBOutlet UIButton *btnPlaySound;
@property (weak, nonatomic) IBOutlet UIButton *btnShowLight;
@property (weak, nonatomic) IBOutlet UILabel *lblRSSI;
@property (strong, nonatomic) BLE *ble;
@property (weak, nonatomic) IBOutlet UILabel *lblConnectBtn;

@end

@implementation TiltViewController

@synthesize ble;
@synthesize btnConnect;
@synthesize btnPlaySound;
@synthesize btnShowLight;
@synthesize lblRSSI;
@synthesize lblConnectBtn;
NSInteger const connectionTimeout = 3;
BOOL lightVal = FALSE;
BOOL soundVal = FALSE;

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
  self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
  if (self) {
  }
  return self;
}

- (void)viewDidLoad
{
  [super viewDidLoad];
  ble = [BLE getInstance];
  ble.delegate = self;
  [btnShowLight setEnabled:NO];
  [btnPlaySound setEnabled:NO];
}

- (void)didReceiveMemoryWarning
{
  [super didReceiveMemoryWarning];
  // Dispose of any resources that can be recreated.
}

#pragma mark - BLE delegate

NSTimer *rssiTimer;

- (void)bleDidDisconnect
{
  NSLog(@"->Disconnected");
  lblConnectBtn.text = @"Connect";
  lblRSSI.text = @"";
  [btnShowLight setEnabled:NO];
  [btnPlaySound setEnabled:NO];
  [rssiTimer invalidate];
}

// When RSSI is changed, this will be called
-(void) bleDidUpdateRSSI:(NSNumber *) rssi
{
  lblRSSI.text = [[NSString alloc] initWithFormat:@"RSSI: %@ dB", rssi.stringValue];
}

-(void) readRSSITimer:(NSTimer *)timer
{
  [ble readRSSI];
}

// When disconnected, this will be called
-(void) bleDidConnect
{
  NSLog(@"->Connected");
  lblConnectBtn.text = @"Disconnect";
  lblRSSI.text = @"";
  
  // Send reset
  UInt8 buf[] = {kResetPins, 0x00, 0x00};
  NSData *data = [[NSData alloc] initWithBytes:buf length:3];
  [ble write:data];
  
  // Schedule to read RSSI every 1 sec.
  rssiTimer = [NSTimer scheduledTimerWithTimeInterval:(float)1.0 target:self selector:@selector(readRSSITimer:) userInfo:nil repeats:YES];
}

// When data is coming, this will be called
- (void)bleDidReceiveData:(unsigned char *)data length:(int)length
{
  BleCmdTiltToPhone cmdId = (BleCmdTiltToPhone)data[0];
  unsigned char *rcvdData = data;
  NSLog(@"bleDidReceiveData: cmd %lu length: %d", cmdId, length);
  
  rcvdData++;
  switch (cmdId) {
    case kGeneralMsg:
      {
        NSString *rcvdMsg = [[NSString alloc] initWithCString:rcvdData encoding:NSASCIIStringEncoding];
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"T/LT"
                                                        message:rcvdMsg
                                                       delegate:nil
                                              cancelButtonTitle:@"OK"
                                              otherButtonTitles:nil];
        [alert show];
      }
      break;
    default:
      break;
  }
  // parse data, all commands are in 3-byte
//  for (int i = 0; i < length; i+=3) {
//    NSLog(@"0x%02X, 0x%02X, 0x%02X", data[i], data[i+1], data[i+2]);
//  }
}

- (void)bleDidChangedStateToPoweredOn
{
  [self initiateConnection];
}

- (void)initiateConnection
{
  // Peripherals actively in use. Cancel the connection in order to create a new connection.
  if (ble.activePeripheral) {
    if (ble.activePeripheral.state == CBPeripheralStateConnected) {
      [[ble CM] cancelPeripheralConnection:[ble activePeripheral]];
      lblConnectBtn.text = @"Connect";
      return;
    }
  }
  
  // Set the list of existing peripherals to nil
  if (ble.peripherals) {
    ble.peripherals = nil;
  }
  
  // Try to find peripherals for the next 2 seconds
  if ([ble findBLEPeripherals:connectionTimeout] != -1) {
    [btnConnect setEnabled:false];
    lblConnectBtn.text = @"Connecting ...";
    // Start connection timer for connectionTimeout value
    [NSTimer scheduledTimerWithTimeInterval:(float)connectionTimeout target:self selector:@selector(connectionTimer:) userInfo:nil repeats:NO];
  } else {
    [btnShowLight setEnabled:NO];
    [btnPlaySound setEnabled:NO];
    lblConnectBtn.text = @"Connect";    
    UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"T/LT"
                                                    message:@"Please make sure Bluetooth is enabled from the Settings App."
                                                   delegate:nil
                                          cancelButtonTitle:@"Dismiss"
                                          otherButtonTitles:nil];
    [alert show];
  }
}

- (IBAction)scanForTilt:(id)sender forEvent:(UIEvent *)event {
  [self initiateConnection];
}

- (IBAction)playSoundToFindBike:(id)sender forEvent:(UIEvent *)event
{
  UInt8 buf[3] = {kPlaySound, 0x00 , 0x00};
  if (soundVal == FALSE) {
    buf[1] = TRUE;
    soundVal = TRUE;
  } else {
    soundVal = FALSE;
  }
  NSData *data = [[NSData alloc] initWithBytes:buf length:3];
  [ble write:data];
}

- (IBAction)showLightToFindBike:(id)sender forEvent:(UIEvent *)event
{
  UInt8 buf[3] = {kTurnOnLight, 0x00 , 0x00};
  if (lightVal == FALSE) {
    buf[1] = TRUE;
    lightVal = TRUE;
  } else {
    lightVal = FALSE;
  }
  NSData *data = [[NSData alloc] initWithBytes:buf length:3];
  [ble write:data];
}

- (void)connectionTimer:(NSTimer *)timer
{
  [btnConnect setEnabled:true];
  
  if (ble.peripherals.count > 0) {
    lblConnectBtn.text = @"Disconnect";
    [btnShowLight setEnabled:YES];
    [btnPlaySound setEnabled:YES];
    [ble connectPeripheral:[ble.peripherals objectAtIndex:0]];
  }
  else {
    lblConnectBtn.text = @"Connect";
    [btnShowLight setEnabled:NO];
    [btnPlaySound setEnabled:NO];
    UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"T/LT"
                                                    message:@"Could not find your T/LT device nearby. Please try again when in range."
                                                   delegate:nil
                                          cancelButtonTitle:@"Dismiss"
                                          otherButtonTitles:nil];
    [alert show];
  }
}

@end
