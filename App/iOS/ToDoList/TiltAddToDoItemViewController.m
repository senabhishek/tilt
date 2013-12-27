//
//  TiltAddToDoItemViewController.m
//  ToDoList
//
//  Created by Abhishek Sen on 12/24/13.
//  Copyright (c) 2013 Tilt. All rights reserved.
//

#import "TiltAddToDoItemViewController.h"

@interface TiltAddToDoItemViewController ()
@property (weak, nonatomic) IBOutlet UITextField *textField;
@property (weak, nonatomic) IBOutlet UIBarButtonItem *doneButton;

@end

@implementation TiltAddToDoItemViewController

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
	// Do any additional setup after loading the view.
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
  if (sender != self.doneButton) return;
  
  if (self.textField.text.length > 0) {
    self.toDoItem = [[TiltToDoItem alloc] init];
    self.toDoItem.itemName = self.textField.text;
    self.toDoItem.creationDate = [NSDate date];
    self.toDoItem.completed = NO;
  }
}

@end
