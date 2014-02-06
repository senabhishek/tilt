//
//  TiltConnectedController.m
//  Tilt
//
//  Created by Abhishek Sen on 12/27/13.
//  Copyright (c) 2013 Tilt. All rights reserved.
//

#import "TiltConnectedController.h"
#import "BLE.h"
#import "BLEDefines.h"
#import <CoreLocation/CoreLocation.h>

@interface TiltConnectedController ()
@property (weak, nonatomic) IBOutlet UIButton *btnPlaySound;
@property (weak, nonatomic) IBOutlet UIButton *btnShowLight;
@property (weak, nonatomic) IBOutlet UILabel *lblRSSI;
@property (strong, nonatomic) BLE *ble;
@property (strong, nonatomic) CLLocationManager *locationManager;
@end

@implementation TiltConnectedController

@synthesize ble;
@synthesize btnPlaySound;
@synthesize btnShowLight;
@synthesize lblRSSI;
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
//  self.locationManager = [[CLLocationManager alloc] init];
//  self.locationManager.desiredAccuracy = kCLLocationAccuracyBest;
//  [self.locationManager startUpdatingLocation];
}

- (void)didReceiveMemoryWarning
{
  [super didReceiveMemoryWarning];
  // Dispose of any resources that can be recreated.
}

#pragma mark - BLE delegate

//NSTimer *rssiTimer;

- (void)bleDidDisconnect
{
  NSLog(@"->Disconnected");
//  lblConnectBtn.text = @"Connect";
//  lblRSSI.text = @"";
//  [btnShowLight setEnabled:NO];
//  [btnPlaySound setEnabled:NO];
//  [rssiTimer invalidate];
  [self goBackToMainView];
}

// When RSSI is changed, this will be called
//-(void) bleDidUpdateRSSI:(NSNumber *) rssi
//{
////  lblRSSI.text = [[NSString alloc] initWithFormat:@"RSSI: %@ dB", rssi.stringValue];
//}

//-(void) readRSSITimer:(NSTimer *)timer
//{
//  [ble readRSSI];
//}

// When disconnected, this will be called
//-(void) bleDidConnect
//{
//  NSLog(@"->Connected");
////  lblConnectBtn.text = @"Disconnect";
//  lblRSSI.text = @"";
//  
//  // Send reset
//  UInt8 buf[] = {kResetPins, 0x00, 0x00};
//  NSData *data = [[NSData alloc] initWithBytes:buf length:3];
//  [ble write:data];
//  
//  // Schedule to read RSSI every 1 sec.
//  rssiTimer = [NSTimer scheduledTimerWithTimeInterval:(float)1.0 target:self selector:@selector(readRSSITimer:) userInfo:nil repeats:YES];
//}

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
}

//- (void)bleDidChangedStateToPoweredOn
//{
//  [self initiateConnection];
//}

//- (void)initiateConnection
//{
//  // Peripherals actively in use. Cancel the connection in order to create a new connection.
//  if (ble.activePeripheral) {
//    if (ble.activePeripheral.state == CBPeripheralStateConnected) {
//      [[ble CM] cancelPeripheralConnection:[ble activePeripheral]];
////      lblConnectBtn.text = @"Connect";
//      return;
//    }
//  }
//  
//  // Set the list of existing peripherals to nil
//  if (ble.peripherals) {
//    ble.peripherals = nil;
//  }
//  
//  // Try to find peripherals for the next 2 seconds
//  if ([ble findBLEPeripherals:connectionTimeout] != -1) {
////    [btnConnect setEnabled:false];
////    lblConnectBtn.text = @"Connecting ...";
//    // Start connection timer for connectionTimeout value
//    [NSTimer scheduledTimerWithTimeInterval:(float)connectionTimeout target:self selector:@selector(connectionTimer:) userInfo:nil repeats:NO];
//  } else {
//    [btnShowLight setEnabled:NO];
//    [btnPlaySound setEnabled:NO];
////    lblConnectBtn.text = @"Connect";    
//    UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"T/LT"
//                                                    message:@"Please make sure Bluetooth is enabled from the Settings App."
//                                                   delegate:nil
//                                          cancelButtonTitle:@"Dismiss"
//                                          otherButtonTitles:nil];
//    [alert show];
//  }
//}

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

//- (void)connectionTimer:(NSTimer *)timer
//{
////  [btnConnect setEnabled:true];
//  
//  if (ble.peripherals.count > 0) {
////    lblConnectBtn.text = @"Disconnect";
//    [btnShowLight setEnabled:YES];
//    [btnPlaySound setEnabled:YES];
//    [ble connectPeripheral:[ble.peripherals objectAtIndex:0]];
//  }
//  else {
////    lblConnectBtn.text = @"Connect";
////    [btnShowLight setEnabled:NO];
////    [btnPlaySound setEnabled:NO];
////    UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"T/LT"
////                                                    message:@"Could not find your T/LT device nearby. Please try again when in range."
////                                                   delegate:nil
////                                          cancelButtonTitle:@"Dismiss"
////                                          otherButtonTitles:nil];
////    [alert show];
//    [self goBackToMainView];
//  }
//}

- (void)goBackToMainView
{
  SEL theUnwindSelector = @selector(goBackToMainView:);
  NSString *unwindSegueIdentifier = @"unwindToMainViewSegue";
  
  UINavigationController *nc = [self navigationController];
  // Find the view controller that has this unwindAction selector (may not be one in the nav stack)
  UIViewController *viewControllerToCallUnwindSelectorOn = [nc viewControllerForUnwindSegueAction: theUnwindSelector
                                                                               fromViewController: self
                                                                                       withSender: nil];
  // None found, then do nothing.
  if (viewControllerToCallUnwindSelectorOn == nil) {
    NSLog(@"No controller found to unwind too");
    return;
  }
  
  // Can the controller that we found perform the unwind segue.  (This is decided by that controllers implementation of canPerformSeque: method
  BOOL cps = [viewControllerToCallUnwindSelectorOn canPerformUnwindSegueAction:theUnwindSelector
                                                            fromViewController: self
                                                                    withSender: nil];
  // If we have permission to perform the seque on the controller where the unwindAction is implmented
  // then get the segue object and perform it.
  if (cps) {
    UIStoryboardSegue *unwindSegue = [nc segueForUnwindingToViewController: viewControllerToCallUnwindSelectorOn
                                                        fromViewController: self
                                                                identifier: unwindSegueIdentifier];
    [viewControllerToCallUnwindSelectorOn prepareForSegue: unwindSegue sender: self];
    [unwindSegue perform];
  }
}
@end
