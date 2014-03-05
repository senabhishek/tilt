//
//  TiltConnectedController.m
//  Tilt
//
//  Created by Abhishek Sen on 12/27/13.
//  Copyright (c) 2013 Tilt. All rights reserved.
//

#import "TiltConnectedController.h"
#import "TiltMainViewController.h"
#import "BLE.h"
#import "BLEDefines.h"
#import <CoreLocation/CoreLocation.h>

@interface TiltConnectedController ()
@property (weak, nonatomic) IBOutlet UIButton *btnPlaySound;
@property (weak, nonatomic) IBOutlet UIButton *btnShowLight;
@property (weak, nonatomic) IBOutlet UILabel *lblRSSI;
@property (weak, nonatomic) IBOutlet UIButton *btnDisconnect;
@property (weak, nonatomic) IBOutlet UILabel *lblDisconnect;
@property (weak, nonatomic) IBOutlet UIButton *btnSoundLight;
@property (strong, nonatomic) BLE *ble;
@property (strong, nonatomic) CLLocationManager *locationManager;
@end

@implementation TiltConnectedController
@synthesize ble;
@synthesize btnPlaySound;
@synthesize btnShowLight;
@synthesize btnSoundLight;
@synthesize lblRSSI;
BOOL lightVal = FALSE;
BOOL soundVal = FALSE;
BOOL lightSoundVal = FALSE;
NSString *welcomeMsg = @"Welcome to T/LT!";
NSString *bikeTheftNotfMsg = @"Your bike is being moved!!!";

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
  ble.ble_delegate = self;
//  self.locationManager = [[CLLocationManager alloc] init];
//  self.locationManager.desiredAccuracy = kCLLocationAccuracyBest;
//  [self.locationManager startUpdatingLocation];
  [self.navigationItem setHidesBackButton:YES];
  [[self navigationController] setNavigationBarHidden:YES animated:YES];
  UIImageView *bgImageView = [[UIImageView alloc] initWithImage:[UIImage imageNamed:@"background"]];
  bgImageView.frame = self.view.bounds;
  bgImageView.contentMode = UIViewContentModeScaleAspectFill;
  bgImageView.alpha = 0.9;  
  [self.view addSubview:bgImageView];
  [self.view sendSubviewToBack:bgImageView];
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
  [self goBackToMainView:self];
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
    case kBikeTheftNotfMsg:
      {
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"T/LT"
                                                        message:bikeTheftNotfMsg
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

- (void)sendCommand:(BleCmdPhoneToTilt)cmd
{
  UInt8 buf[] = {cmd, 0x00, 0x00};
  BOOL validCmd = YES;
  
  switch (cmd) {
    case kTurnOnLight:
      if (lightVal == FALSE) {
        buf[1] = TRUE;
        lightVal = TRUE;
        [btnShowLight setImage:[UIImage imageNamed:@"LightOuterglow"] forState:(UIControlStateNormal)];
      } else {
        lightVal = FALSE;
        [btnShowLight setImage:[UIImage imageNamed:@"Light"] forState:(UIControlStateNormal)];
      }
      break;
    case kPlaySound:
      if (soundVal == FALSE) {
        buf[1] = TRUE;
        soundVal = TRUE;
        [btnPlaySound setImage:[UIImage imageNamed:@"EarSoundOuterglow"] forState:(UIControlStateNormal)];
      } else {
        soundVal = FALSE;
        [btnPlaySound setImage:[UIImage imageNamed:@"earSound"] forState:(UIControlStateNormal)];
      }
      break;
    case kResetPins:
      break;
    case kLightSound:
      if (lightSoundVal == FALSE) {
        buf[1] = TRUE;
        lightSoundVal = TRUE;
        [btnSoundLight setImage:[UIImage imageNamed:@"comboShortOutglow"] forState:(UIControlStateNormal)];
      } else {
        lightSoundVal = FALSE;
        [btnSoundLight setImage:[UIImage imageNamed:@"comboShort"] forState:(UIControlStateNormal)];
      }
      break;
    default:
      validCmd = NO;
      break;
  }
  
  if (validCmd) {
    NSData *data = [[NSData alloc] initWithBytes:buf length:sizeof(buf)];
    [ble write:data];
  }
}

- (IBAction)playSoundToFindBike:(id)sender forEvent:(UIEvent *)event
{
  [self sendCommand:kPlaySound];
}

- (IBAction)showLightToFindBike:(id)sender forEvent:(UIEvent *)event
{
  [self sendCommand:kTurnOnLight];
}

- (IBAction)showLightSound:(id)sender
{
  [self sendCommand:kLightSound];
}

- (IBAction)disconnect:(id)sender
{
  if (ble.activePeripheral) {
    if (ble.activePeripheral.state == CBPeripheralStateConnected) {
      // Cancel existing operations
      [self sendCommand:kResetPins];
      // Disconnect from device
      [[ble CM] cancelPeripheralConnection:[ble activePeripheral]];
    }
  }
}

- (void)goBackToMainView: (id)sender
{
  SEL theUnwindSelector = @selector(goBackToMainView:);
  NSString *unwindSegueIdentifier = @"unwindToMainViewSegue";
  
  UINavigationController *nc = [self navigationController];
  // Find the view controller that has this unwindAction selector (may not be one in the nav stack)
  UIViewController *viewControllerToCallUnwindSelectorOn = [nc viewControllerForUnwindSegueAction: theUnwindSelector
                                                                               fromViewController: self
                                                                                       withSender: sender];
  // None found, then do nothing.
  if (viewControllerToCallUnwindSelectorOn == nil) {
    NSLog(@"No controller found to unwind too");
    return;
  }
  
  // Can the controller that we found perform the unwind segue.  (This is decided by that controllers implementation of canPerformSeque: method
  BOOL canPerformUnwind = [viewControllerToCallUnwindSelectorOn canPerformUnwindSegueAction:theUnwindSelector
                                                            fromViewController: self
                                                                    withSender: sender];
  // If we have permission to perform the seque on the controller where the unwindAction is implmented
  // then get the segue object and perform it.
  if (canPerformUnwind) {
    UIStoryboardSegue *unwindSegue = [nc segueForUnwindingToViewController: viewControllerToCallUnwindSelectorOn
                                                        fromViewController: self
                                                                identifier: unwindSegueIdentifier];
    TiltMainViewController *mainVC = (TiltMainViewController *)viewControllerToCallUnwindSelectorOn;
    [mainVC setConnectLabelText:@"Connect"];
    [viewControllerToCallUnwindSelectorOn prepareForSegue: unwindSegue sender: self];
    [unwindSegue perform];
  }
}

@end
