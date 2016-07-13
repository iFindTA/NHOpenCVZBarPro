//
//  NHVideoCamera.h
//  NHOpenCVPro
//
//  Created by hu jiaju on 15/10/13.
//  Copyright © 2015年 hu jiaju. All rights reserved.
//

#import <opencv2/highgui/cap_ios.h>

@protocol VideoCameraDelegate <CvVideoCameraDelegate>

@end

@interface NHVideoCamera : CvVideoCamera

- (void)updateOrientation;
- (void)layoutPreviewLayer;

@end
