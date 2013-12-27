//
//  TiltToDoItem.m
//  ToDoList
//
//  Created by Abhishek Sen on 12/24/13.
//  Copyright (c) 2013 Tilt. All rights reserved.
//

#import "TiltToDoItem.h"

@interface TiltToDoItem ()

@property NSDate *completionDate;

@end

@implementation TiltToDoItem

- (void)markAsCompleted:(BOOL)isComplete
{
  self.completed = isComplete;
  [self setCompletionDate];
}

- (void)setCompletionDate {
  if (self.completed) {
    self.completionDate = [NSDate date];
  } else {
    self.completionDate = nil;
  }
}

@end
