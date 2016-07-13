//
//  NHScanVCR.h
//  NHOCRScanPro
//
//  Created by hu jiaju on 15/7/30.
//  Copyright (c) 2015年 hu jiaju. All rights reserved.
//

#import <UIKit/UIKit.h>

typedef void(^NHScanResponse)(BOOL scan,NSString *info);

typedef enum : NSUInteger {
    NHScanTypeNone,
    NHScanTypeQR,
    NHScanTypeID,
    NHScanTypeBank,
} NHScanType;

@interface NHScanVCR : UIViewController

- (nonnull instancetype)initWithScanType:(NHScanType)type;

-(void)handelScanResponse:(NHScanResponse)response;

@end
