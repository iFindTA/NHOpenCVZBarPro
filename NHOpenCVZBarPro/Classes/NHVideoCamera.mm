//
//  NHVideoCamera.m
//  NHOpenCVPro
//
//  Created by hu jiaju on 15/10/13.
//  Copyright © 2015年 hu jiaju. All rights reserved.
//

#import "NHVideoCamera.h"

#define DEGREES_RADIANS(angle) ((angle) / 180.0 * M_PI)

@implementation NHVideoCamera

- (void)updateOrientation {
    self->customPreviewLayer.bounds = CGRectMake(0, 0, self.parentView.frame.size.width, self.parentView.frame.size.height);
    [self layoutPreviewLayer];
}

- (void)layoutPreviewLayer {
    if (self.parentView != nil) {
        
        CALayer* layer = self->customPreviewLayer;
        CGRect bounds = self->customPreviewLayer.bounds;
        int rotation_angle = 0;
        
        switch (defaultAVCaptureVideoOrientation) {
            case AVCaptureVideoOrientationLandscapeRight:
                rotation_angle = 270;
                break;
            case AVCaptureVideoOrientationPortraitUpsideDown:
                rotation_angle = 180;
                break;
            case AVCaptureVideoOrientationLandscapeLeft:
                rotation_angle = 90;
                break;
            case AVCaptureVideoOrientationPortrait:
            default:
                break;
        }
        
        layer.position = CGPointMake(self.parentView.frame.size.width/2., self.parentView.frame.size.height/2.);
        layer.affineTransform = CGAffineTransformMakeRotation( DEGREES_RADIANS(rotation_angle) );
        layer.bounds = bounds;
    }
}

@end
