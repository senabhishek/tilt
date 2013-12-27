//
//  TiltViewController.m
//  ToDoList
//
//  Created by Abhishek Sen on 12/24/13.
//  Copyright (c) 2013 Tilt. All rights reserved.
//

#import "TiltViewController.h"
#import "TiltToDoItem.h"
#import "TiltAddToDoItemViewController.h"

@interface TiltViewController ()

@property NSMutableArray *toDoItems;

@end

@implementation TiltViewController

- (IBAction)unwindToDoList:(UIStoryboardSegue *)segue
{
  TiltAddToDoItemViewController *source = [segue sourceViewController];
  if (source.toDoItem) {
    TiltToDoItem *item = source.toDoItem;
    [self.toDoItems addObject:item];
    [self.tableView reloadData];
  }
  
  
}

- (id)initWithStyle:(UITableViewStyle)style
{
    self = [super initWithStyle:style];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    self.toDoItems = [[NSMutableArray alloc] init];
    [self loadInitialData];
    // Uncomment the following line to preserve selection between presentations.
    // self.clearsSelectionOnViewWillAppear = NO;
 
    // Uncomment the following line to display an Edit button in the navigation bar for this view controller.
    // self.navigationItem.rightBarButtonItem = self.editButtonItem;
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

#pragma mark - Table view data source

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView
{
#warning Potentially incomplete method implementation.
    // Return the number of sections.
    return 1;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
#warning Incomplete method implementation.
    // Return the number of rows in the section.
    return [self.toDoItems count];
}

- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath
{
    static NSString *CellIdentifier = @"ListPrototypeCell";
    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:CellIdentifier forIndexPath:indexPath];
    TiltToDoItem *toDoItem = [self.toDoItems objectAtIndex:indexPath.row];
    cell.textLabel.text = toDoItem.itemName;
    if (toDoItem.completed){
      cell.accessoryType = UITableViewCellAccessoryCheckmark;
    } else {
      cell.accessoryType = UITableViewCellAccessoryNone;
    }
    return cell;
}

- (void)loadInitialData {
  TiltToDoItem *item1 = [[TiltToDoItem alloc] init];
  item1.itemName = @"Buy milk";
  [self.toDoItems addObject:item1];
  TiltToDoItem *item2 = [[TiltToDoItem alloc] init];
  item2.itemName = @"Sleep";
  [self.toDoItems addObject:item2];
  TiltToDoItem *item3 = [[TiltToDoItem alloc] init];
  item3.itemName = @"Do homework";
  [self.toDoItems addObject:item3];
}

/*
// Override to support conditional editing of the table view.
- (BOOL)tableView:(UITableView *)tableView canEditRowAtIndexPath:(NSIndexPath *)indexPath
{
    // Return NO if you do not want the specified item to be editable.
    return YES;
}
*/

/*
// Override to support editing the table view.
- (void)tableView:(UITableView *)tableView commitEditingStyle:(UITableViewCellEditingStyle)editingStyle forRowAtIndexPath:(NSIndexPath *)indexPath
{
    if (editingStyle == UITableViewCellEditingStyleDelete) {
        // Delete the row from the data source
        [tableView deleteRowsAtIndexPaths:@[indexPath] withRowAnimation:UITableViewRowAnimationFade];
    }   
    else if (editingStyle == UITableViewCellEditingStyleInsert) {
        // Create a new instance of the appropriate class, insert it into the array, and add a new row to the table view
    }   
}
*/

/*
// Override to support rearranging the table view.
- (void)tableView:(UITableView *)tableView moveRowAtIndexPath:(NSIndexPath *)fromIndexPath toIndexPath:(NSIndexPath *)toIndexPath
{
}
*/

/*
// Override to support conditional rearranging of the table view.
- (BOOL)tableView:(UITableView *)tableView canMoveRowAtIndexPath:(NSIndexPath *)indexPath
{
    // Return NO if you do not want the item to be re-orderable.
    return YES;
}
*/

/*
#pragma mark - Navigation

// In a story board-based application, you will often want to do a little preparation before navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    // Get the new view controller using [segue destinationViewController].
    // Pass the selected object to the new view controller.
}

 */

#pragma mark - Table view delegate
- (void)tableView:(UITableView *)tableView didSelectRowAtIndexPath:(NSIndexPath *)indexPath
{
  [tableView deselectRowAtIndexPath:indexPath animated:NO];
  TiltToDoItem *tappedItem = [self.toDoItems objectAtIndex:indexPath.row];
  [tappedItem markAsCompleted:!tappedItem.completed];
  [tableView reloadRowsAtIndexPaths:@[indexPath] withRowAnimation:UITableViewRowAnimationNone];
  
}


@end
