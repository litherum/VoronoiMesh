//
//  GameViewController.h
//  VoronoiMesh
//
//  Created by Litherum on 9/20/14.
//  Copyright (c) 2014 Litherum. All rights reserved.
//

#import <SceneKit/SceneKit.h>

#import "GameView.h"

@interface GameViewController : NSViewController <SCNProgramDelegate>
@property (assign) IBOutlet GameView *gameView;
@end
