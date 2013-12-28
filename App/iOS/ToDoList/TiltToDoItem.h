//
//  TiltToDoItem.h
//  ToDoList
//
//  Created by Abhishek Sen on 12/24/13.
//  Copyright (c) 2013 Tilt. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface TiltToDoItem : NSObject
	
@property NSString *itemName;
@property BOOL completed;
@property NSDate *creationDate;
- (void)markAsCompleted:(BOOL)isComplete;
@end
