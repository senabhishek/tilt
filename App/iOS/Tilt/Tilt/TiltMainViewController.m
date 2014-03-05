//
//  TiltMainViewController.m
//  Tilt
//
//  Created by Abhishek Sen on 2/4/14.
//  Copyright (c) 2014 Tilt. All rights reserved.
//

#import "TiltMainViewController.h"
#import "TiltConnectedController.h"
#import "BLE.h"
#import "BLEDefines.h"

@interface TiltMainViewController ()
@property (weak, nonatomic) IBOutlet UIButton *btnConnect;
@property (weak, nonatomic) IBOutlet UILabel *lblConnectBtn;
- (IBAction)connect:(UIButton *)sender;
@property (weak, nonatomic) IBOutlet UIActivityIndicatorView *actIndicatorConnecting;
@property (strong, nonatomic) BLE *ble;
@end

@implementation TiltMainViewController
@synthesize btnConnect;
@synthesize lblConnectBtn;
@synthesize actIndicatorConnecting;
@synthesize ble;

NSInteger const connectionTimeout = 2;
NSString *appNameString = @"T/LT";
NSString *connectString = @"Connect";
NSString *connectingString = @"Connecting ...";
NSString *btDisabledString = @"Please make sure Bluetooth is enabled from the Settings App.";
NSString *outOfRangeString = @"Could not find your T/LT device nearby. Please try again when in range.";

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
  self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
  if (self) {
      // Custom initialization
  }
  return self;
}

- (void)viewDidLoad
{
  [super viewDidLoad];
  ble = [BLE getInstance];
  ble.ble_connect_delegate = self;
  [[self navigationController] setNavigationBarHidden:YES animated:YES];
  UIImageView *bgImageView = [[UIImageView alloc] initWithImage:[UIImage imageNamed:@"background"]];
  bgImageView.frame = self.view.bounds;
  bgImageView.contentMode = UIViewContentModeScaleAspectFill;
  bgImageView.alpha = 0.9;
  [self.view addSubview:bgImageView];
  [self.view sendSubviewToBack:bgImageView];
  actIndicatorConnecting.hidden = YES;
}

- (void)didReceiveMemoryWarning
{
  [super didReceiveMemoryWarning];
  // Dispose of any resources that can be recreated.
}

-(void) bleDidConnect
{
  NSLog(@"->Connected");
  
  // Send reset
  UInt8 buf[] = {kResetPins, 0x00, 0x00};
  NSData *data = [[NSData alloc] initWithBytes:buf length:3];
  [ble write:data];
  [actIndicatorConnecting stopAnimating];
  actIndicatorConnecting.hidden = YES;
  [btnConnect setEnabled:true];
  [self performSegueWithIdentifier: @"segueToTiltConnectedVC" sender: self];
}

- (void)bleDidChangedStateToPoweredOn
{
  NSLog(@"bleDidChangedStateToPoweredOn: BT ready!");
}

- (IBAction)connect:(UIButton *)sender
{
  [self initiateConnection];
}

- (void)initiateConnection
{
  // Peripherals actively in use. Cancel the connection in order to create a new connection.
  if (ble.activePeripheral) {
    if (ble.activePeripheral.state == CBPeripheralStateConnected) {
      [[ble CM] cancelPeripheralConnection:[ble activePeripheral]];
      lblConnectBtn.text = connectString;
      return;
    }
  }
  
  // Set the list of existing peripherals to nil
  if (ble.peripherals) {
    ble.peripherals = nil;
  }
  
  // Try to find peripherals for the next 2 seconds
  if ([ble findBLEPeripherals:connectionTimeout] != -1) {
    lblConnectBtn.text = connectingString;
    [btnConnect setEnabled:false];
    
    // Start connection timer for connectionTimeout value
    [NSTimer scheduledTimerWithTimeInterval:(float)connectionTimeout target:self selector:@selector(connectionTimer:) userInfo:nil repeats:NO];
    [actIndicatorConnecting startAnimating];
    actIndicatorConnecting.hidden = NO;
  } else {
    lblConnectBtn.text = connectString;
    UIAlertView *alert = [[UIAlertView alloc] initWithTitle:appNameString
                                                    message:btDisabledString
                                                   delegate:nil
                                          cancelButtonTitle:@"Dismiss"
                                          otherButtonTitles:nil];
    [alert show];
  }
}

- (void)connectionTimer:(NSTimer *)timer
{
 
  if (ble.peripherals.count > 0) {
    [ble connectPeripheral:[ble.peripherals objectAtIndex:0]];
  }
  else {
    [btnConnect setEnabled:true];
    lblConnectBtn.text = connectString;
    UIAlertView *alert = [[UIAlertView alloc] initWithTitle:appNameString
                                                    message:outOfRangeString
                                                   delegate:nil
                                          cancelButtonTitle:@"Dismiss"
                                          otherButtonTitles:nil];
    [alert show];
  }
  [actIndicatorConnecting stopAnimating];
  actIndicatorConnecting.hidden = YES;
}

# pragma mark

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
  if ([[segue identifier] isEqualToString:@"segueToTiltConnectedVC"])
  {
//    UIViewController *destVC = [segue destinationViewController];
//    [destVC performSelector:@selector(setBle:)];
  }
}

- (IBAction)goBackToMainView: (UIStoryboardSegue*)segue
{
  NSLog(@"Called goBackToMainView: unwind action");
}

- (void)setConnectLabelText: (NSString *) textVal
{
  lblConnectBtn.text = textVal;
}

@end
