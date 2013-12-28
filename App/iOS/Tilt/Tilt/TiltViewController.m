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
@property (strong, nonatomic) BLE *ble;

@end

@implementation TiltViewController

@synthesize ble;
@synthesize btnConnect;
@synthesize btnPlaySound;
@synthesize btnShowLight;

NSInteger const connectionTimeout = 2;

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
  }
}


@end
