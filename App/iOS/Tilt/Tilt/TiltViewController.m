//
//  TiltViewController.m
//  Tilt
//
//  Created by Abhishek Sen on 12/27/13.
//  Copyright (c) 2013 Tilt. All rights reserved.
//

#import "TiltViewController.h"
#import "BLE.h"

@interface TiltViewController ()

@property (weak, nonatomic) IBOutlet UIButton *btnConnect;
@property (weak, nonatomic) IBOutlet UIButton *btnPlaySound;
@property (weak, nonatomic) IBOutlet UIButton *btnShowLight;
@property (weak, nonatomic) IBOutlet UILabel *lblRSSI;
@property (strong, nonatomic) BLE *ble;

@end

@implementation TiltViewController

@synthesize ble;
@synthesize btnConnect;
@synthesize btnPlaySound;
@synthesize btnShowLight;
@synthesize lblRSSI;
NSInteger const connectionTimeout = 3;

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
  [self initiateConnection];
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
  
  [btnConnect setTitle:@"Connect" forState:UIControlStateNormal];
  
  lblRSSI.text = @"---"; 
  [rssiTimer invalidate];
}

// When RSSI is changed, this will be called
-(void) bleDidUpdateRSSI:(NSNumber *) rssi
{
  lblRSSI.text = [[NSString alloc] initWithFormat:@"RSSI: %@", rssi.stringValue];
}

-(void) readRSSITimer:(NSTimer *)timer
{
  [ble readRSSI];
}

// When disconnected, this will be called
-(void) bleDidConnect
{
  NSLog(@"->Connected");
  
  // send reset
  UInt8 buf[] = {0x04, 0x00, 0x00};
  NSData *data = [[NSData alloc] initWithBytes:buf length:3];
  [ble write:data];
  
  // Schedule to read RSSI every 1 sec.
  rssiTimer = [NSTimer scheduledTimerWithTimeInterval:(float)1.0 target:self selector:@selector(readRSSITimer:) userInfo:nil repeats:YES];
}

// When data is comming, this will be called
- (void)bleDidReceiveData:(unsigned char *)data length:(int)length
{
  NSLog(@"bleDidReceiveData: Length: %d", length);
  
  // parse data, all commands are in 3-byte
  for (int i = 0; i < length; i+=3) {
    NSLog(@"0x%02X, 0x%02X, 0x%02X", data[i], data[i+1], data[i+2]);
  }
}

- (void)initiateConnection
{
  // Peripherals actively in use. Cancel the connection in order to create a new connection.
  if (ble.activePeripheral) {
    if (ble.activePeripheral.state == CBPeripheralStateConnected) {
      [[ble CM] cancelPeripheralConnection:[ble activePeripheral]];
      [btnConnect setTitle:@"Connect" forState:UIControlStateNormal];
      return;
    }
  }
  
  // Set the list of existing peripherals to nil
  if (ble.peripherals) {
    ble.peripherals = nil;
  }
  
  [btnConnect setEnabled:false];
  
  // Try to find peripherals for the next 2 seconds
  [ble findBLEPeripherals:connectionTimeout];
  
  // Start connection timer for connectionTimeout value
  [NSTimer scheduledTimerWithTimeInterval:(float)connectionTimeout target:self selector:@selector(connectionTimer:) userInfo:nil repeats:NO];
}

- (IBAction)scanForTilt:(id)sender forEvent:(UIEvent *)event {
  [self initiateConnection];
}

- (IBAction)playSoundToFindBike:(id)sender forEvent:(UIEvent *)event
{
  UInt8 buf[3] = {0x01, 0x00, 0x00};
  buf[1] = 0x01;
  NSData *data = [[NSData alloc] initWithBytes:buf length:3];
  [ble write:data];
}

- (IBAction)showLightToFindBike:(id)sender forEvent:(UIEvent *)event
{
  UInt8 buf[3] = {0x02, 0xFF, 0xFF};
  NSData *data = [[NSData alloc] initWithBytes:buf length:3];
  [ble write:data];
}


- (void)connectionTimer:(NSTimer *)timer
{
  [btnConnect setEnabled:true];
  [btnConnect setTitle:@"Disconnect" forState:UIControlStateNormal];
  
  if (ble.peripherals.count > 0) {
    [ble connectPeripheral:[ble.peripherals objectAtIndex:0]];
    [btnShowLight setEnabled:YES];
    [btnPlaySound setEnabled:YES];
  }
  else {
    [btnConnect setTitle:@"Connect" forState:UIControlStateNormal];
    UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"Oops!"
                                                    message:@"Could not find your T/LT device nearby. Please try again when in range."
                                                   delegate:nil
                                          cancelButtonTitle:@"Dismiss"
                                          otherButtonTitles:nil];
    [alert show];
  }
}


@end
