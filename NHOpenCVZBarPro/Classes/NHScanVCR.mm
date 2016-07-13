//
//  NHScanVCR.m
//  NHOCRScanPro
//
//  Created by hu jiaju on 15/7/30.
//  Copyright (c) 2015年 hu jiaju. All rights reserved.
//

#import "NHScanVCR.h"

@interface NHMaskLayer : UIView

/**
 * the rect size for transparent
 */
@property (nonatomic, assign) CGSize transparentArea;

@end

@interface NHMaskLayer ()

@property (nonatomic, strong)UIImageView *t_qr_line_v;
@property (nonatomic, assign)CGFloat t_qr_line_y;

@end
static NSTimeInterval kQrLineanimateDuration = 0.02;
@implementation NHMaskLayer

- (instancetype)initWithFrame:(CGRect)frame {
    self = [super initWithFrame:frame];
    if (self) {
        [self initQRLine];
    }
    return self;
}


- (void)layoutSubviews {
    [super layoutSubviews];
    if (!self.t_qr_line_v) {
        [self initQRLine];
        NSTimer *timer = [NSTimer scheduledTimerWithTimeInterval:kQrLineanimateDuration target:self selector:@selector(show) userInfo:nil repeats:YES];
        [timer fire];
    }
}

-(void)setTransparentArea:(CGSize)transparentArea{
    if (CGSizeEqualToSize(transparentArea, _transparentArea)) {
        return;
    }
    _transparentArea = transparentArea;
    [self setNeedsDisplay];
}

- (void)initQRLine {
    self.t_qr_line_v  = [[UIImageView alloc] initWithFrame:CGRectMake(self.bounds.size.width / 2 - self.transparentArea.width / 2, self.bounds.size.height / 2 - self.transparentArea.height / 2, self.transparentArea.width, 2)];
    self.t_qr_line_v.image = [UIImage imageNamed:@"qr_scan_line"];
    self.t_qr_line_v.contentMode = UIViewContentModeScaleAspectFill;
    [self addSubview:self.t_qr_line_v];
    self.t_qr_line_y = self.t_qr_line_v.frame.origin.y;
}

- (void)show {
    [UIView animateWithDuration:kQrLineanimateDuration animations:^{
        CGRect rect = self.t_qr_line_v.frame;
        rect.origin.y = self.t_qr_line_y;
        self.t_qr_line_v.frame = rect;
    } completion:^(BOOL finished) {
        CGFloat maxBorder = self.frame.size.height / 2 + self.transparentArea.height / 2 - 4;
        if (self.t_qr_line_y > maxBorder) {
            self.t_qr_line_y = self.frame.size.height / 2 - self.transparentArea.height /2;
        }
        self.t_qr_line_y++;
    }];
}

- (void)drawRect:(CGRect)rect {
    //整个二维码扫描界面的颜色
    CGSize screenSize =[UIScreen mainScreen].bounds.size;
    CGRect screenDrawRect =CGRectMake(0, 0, screenSize.width,screenSize.height);
    
    //中间清空的矩形框
    CGRect clearDrawRect = CGRectMake(screenDrawRect.size.width / 2 - self.transparentArea.width / 2,
                                      screenDrawRect.size.height / 2 - self.transparentArea.height / 2,
                                      self.transparentArea.width,self.transparentArea.height);
    
    CGContextRef ctx = UIGraphicsGetCurrentContext();
    [self addScreenFillRect:ctx rect:screenDrawRect];
    [UIView animateWithDuration:1.f delay:0 options:UIViewAnimationOptionBeginFromCurrentState|UIViewAnimationOptionCurveEaseInOut animations:^{
        [self addCenterClearRect:ctx rect:clearDrawRect];
        [self addWhiteRect:ctx rect:clearDrawRect];
        [self addCornerLineWithContext:ctx rect:clearDrawRect];
    } completion:^(BOOL finished) {
        
    }];
}

- (void)addScreenFillRect:(CGContextRef)ctx rect:(CGRect)rect {
    CGContextSetRGBFillColor(ctx, 40 / 255.0,40 / 255.0,40 / 255.0,0.75);
    CGContextFillRect(ctx, rect);   //draw the transparent layer
}

- (void)addCenterClearRect :(CGContextRef)ctx rect:(CGRect)rect {
    CGContextClearRect(ctx, rect);  //clear the center rect  of the layer
}

- (void)addWhiteRect:(CGContextRef)ctx rect:(CGRect)rect {
    CGContextStrokeRect(ctx, rect);
    CGContextSetRGBStrokeColor(ctx, 1, 1, 1, 1);
    CGContextSetLineWidth(ctx, 0.8);
    CGContextAddRect(ctx, rect);
    CGContextStrokePath(ctx);
}

- (void)addCornerLineWithContext:(CGContextRef)ctx rect:(CGRect)rect{
    //画四个边角
    CGContextSetLineWidth(ctx, 2);
    CGContextSetRGBStrokeColor(ctx, 83 /255.0, 239/255.0, 111/255.0, 1);//绿色
    //左上角
    CGPoint poinsTopLeftA[] = {
        CGPointMake(rect.origin.x+0.7, rect.origin.y),
        CGPointMake(rect.origin.x+0.7 , rect.origin.y + 15)
    };
    CGPoint poinsTopLeftB[] = {CGPointMake(rect.origin.x, rect.origin.y +0.7),CGPointMake(rect.origin.x + 15, rect.origin.y+0.7)};
    [self addLine:poinsTopLeftA pointB:poinsTopLeftB ctx:ctx];
    //左下角
    CGPoint poinsBottomLeftA[] = {CGPointMake(rect.origin.x+ 0.7, rect.origin.y + rect.size.height - 15),CGPointMake(rect.origin.x +0.7,rect.origin.y + rect.size.height)};
    CGPoint poinsBottomLeftB[] = {CGPointMake(rect.origin.x , rect.origin.y + rect.size.height - 0.7) ,CGPointMake(rect.origin.x+0.7 +15, rect.origin.y + rect.size.height - 0.7)};
    [self addLine:poinsBottomLeftA pointB:poinsBottomLeftB ctx:ctx];
    //右上角
    CGPoint poinsTopRightA[] = {CGPointMake(rect.origin.x+ rect.size.width - 15, rect.origin.y+0.7),CGPointMake(rect.origin.x + rect.size.width,rect.origin.y +0.7 )};
    CGPoint poinsTopRightB[] = {CGPointMake(rect.origin.x+ rect.size.width-0.7, rect.origin.y),CGPointMake(rect.origin.x + rect.size.width-0.7,rect.origin.y + 15 +0.7 )};
    [self addLine:poinsTopRightA pointB:poinsTopRightB ctx:ctx];
    CGPoint poinsBottomRightA[] = {CGPointMake(rect.origin.x+ rect.size.width -0.7 , rect.origin.y+rect.size.height+ -15),CGPointMake(rect.origin.x-0.7 + rect.size.width,rect.origin.y +rect.size.height )};
    CGPoint poinsBottomRightB[] = {CGPointMake(rect.origin.x+ rect.size.width - 15 , rect.origin.y + rect.size.height-0.7),CGPointMake(rect.origin.x + rect.size.width,rect.origin.y + rect.size.height - 0.7 )};
    [self addLine:poinsBottomRightA pointB:poinsBottomRightB ctx:ctx];
    CGContextStrokePath(ctx);
}

- (void)addLine:(CGPoint[])pointA pointB:(CGPoint[])pointB ctx:(CGContextRef)ctx {
    CGContextAddLines(ctx, pointA, 2);
    CGContextAddLines(ctx, pointB, 2);
}

@end

#pragma mark -- Scan View --

#define NH_CARD_WIDTH_MAX    85.5f
#define NH_CARD_WIDTH_MIN
#define NH_CARD_HEIGHT_MAX   54.0f
#define NH_CARD_HEIGHT_MIN

#define NH_QRCODE_Ratio      0.65

#import <AudioToolbox/AudioToolbox.h>
#import "NHVideoCamera.h"
#import "qr_cutout.hpp"
#import "ocr_card.hpp"
#import "svm_train.hpp"
#import "NHMatImgUtil.h"

using namespace cv;

@interface NHScanVCR ()<CvVideoCameraDelegate, UIAlertViewDelegate>

@property (nonatomic, assign)NHScanType mScanType;
@property (nonatomic, copy)NHScanResponse t_res_response;
@property (nonatomic, strong)NHMaskLayer *t_mask_layer;
@property (nonatomic, strong)UIImageView *preLayer,*t_pre_imgV;
@property (nonatomic, strong)UILabel *t_ocr_label;
@property (nonatomic)CGSize t_mask_size;
@property (nonatomic, strong)NHVideoCamera *camera;
@property (nonatomic)BOOL isScanning;

@end

@implementation NHScanVCR

- (void)dealloc{
    
#if !__has_feature(objc_arc)
    [super dealloc];
#endif
}

- (nonnull instancetype)initWithScanType:(NHScanType)type {
    self = [super init];
    if (self) {
        self.mScanType = type;
    };
    return self;
}

-(void)viewDidLoad{
    [super viewDidLoad];
    
    self.title = @"Scanning";
    self.view.backgroundColor = [UIColor whiteColor];
    
    NSFileManager *fileManager = [NSFileManager defaultManager];
    NSString *filePath = [[NSBundle mainBundle] pathForResource:@"Trains" ofType:@"bundle"];
    NSURL *bundleURL = [NSURL fileURLWithPath:filePath];
    NSArray *contents = [fileManager contentsOfDirectoryAtURL:bundleURL
                                   includingPropertiesForKeys:@[]
                                                      options:NSDirectoryEnumerationSkipsHiddenFiles
                                                        error:nil];
    
    vector< vector<string> > paths;
    for (NSURL *tmpPath in contents) {
        NSArray *datas = [fileManager contentsOfDirectoryAtURL:tmpPath
                                    includingPropertiesForKeys:@[]
                                                       options:NSDirectoryEnumerationSkipsHiddenFiles
                                                         error:nil];
        vector<string> tmpPaths;
        for (NSURL *name in datas) {
            NSString *filePath = [name absoluteString];
            if ([filePath rangeOfString:@"file://"].location != NSNotFound) {
                filePath = [filePath stringByReplacingOccurrencesOfString:@"file://" withString:@""];
            }
            string cPath = [filePath UTF8String];
            tmpPaths.push_back(cPath);
        }
        paths.push_back(tmpPaths);
    }
    
    /// train svm
    svm_trainForPath(paths);
}

- (void)vibrate{
    AudioServicesPlaySystemSound(kSystemSoundID_Vibrate);
}

-(void)viewWillAppear:(BOOL)animated{
    [super viewWillAppear:animated];
    [self setupCamera];
}

-(void)viewDidAppear:(BOOL)animated{
    [super viewDidAppear:animated];
}

-(AVCaptureVideoOrientation)getVideoOrientation{
    return AVCaptureVideoOrientationPortrait;
}

- (void)viewWillDisappear:(BOOL)animated{
    [super viewWillDisappear:animated];
    if (_camera) {
        [_camera stop];
        _isScanning = false;
    }
}

- (void)setMaskLayer{
    if (_mScanType == NHScanTypeQR) {
        int widthScreen = [UIScreen mainScreen].bounds.size.width;
        _t_mask_size = CGSizeMake(widthScreen*NH_QRCODE_Ratio, widthScreen*NH_QRCODE_Ratio);
    }else if (_mScanType == NHScanTypeID|| _mScanType == NHScanTypeBank){
        CGFloat t_wh_scale = NH_CARD_WIDTH_MAX/NH_CARD_HEIGHT_MAX;
        CGFloat t_w = 280;
        CGFloat t_h = t_w/t_wh_scale;
        _t_mask_size = CGSizeMake(t_w,t_h);
    }
    
    if (_t_mask_layer != nil) {
        [_t_mask_layer removeFromSuperview];
        _t_mask_layer = nil;
    }
    CGRect infoRect = self.view.bounds;
    /// update mask
    NHMaskLayer *maskLayer = [[NHMaskLayer alloc] initWithFrame:infoRect];
    maskLayer.backgroundColor = [UIColor clearColor];
    [maskLayer setTransparentArea:_t_mask_size];
    [self.view addSubview:maskLayer];
    _t_mask_layer = maskLayer;
}

-(void)setupCamera {
    CGRect infoRect = self.view.bounds;
    
    //*
    _preLayer = [[UIImageView alloc] initWithFrame:infoRect];
    _preLayer.backgroundColor = [UIColor lightGrayColor];
    _preLayer.contentMode = UIViewContentModeScaleAspectFit;
    [self.view.viewForBaselineLayout addSubview:_preLayer];
    
    //setting camera
    _camera = [[NHVideoCamera alloc] initWithParentView:self.preLayer];
    _camera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    _camera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset1920x1080;
    _camera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait;
    _camera.useAVCaptureVideoPreviewLayer = true;
    _camera.defaultFPS = 30;
    _camera.grayscaleMode = false;
    _camera.delegate = self;
    AVCaptureConnection *connection = _camera.videoCaptureConnection;
    CGFloat maxScale = connection.videoMaxScaleAndCropFactor;
    maxScale /= 50;
    maxScale = MAX(1.0, maxScale);
    NSLog(@"video scale:%f",maxScale);
    connection.videoScaleAndCropFactor = maxScale;
    [_camera setVideoCaptureConnection:connection];
    [_camera layoutPreviewLayer];
    [_camera updateOrientation];
    
    infoRect = CGRectMake(0, 350, 320, 250);
    _t_ocr_label = [[UILabel alloc] initWithFrame:infoRect];
    _t_ocr_label.textColor = [UIColor redColor];
    _t_ocr_label.textAlignment = NSTextAlignmentCenter;
    _t_ocr_label.font = [UIFont systemFontOfSize:20];
    [self.view addSubview:_t_ocr_label];
    
    if (_camera) {
        [_camera start];
        _isScanning = true;
    }
    
    [self setMaskLayer];
}

- (UIImageView *)t_pre_imgV{
    if (!_t_pre_imgV) {
        CGRect infoRect ;
        infoRect.origin = CGPointMake(10, 64);
        infoRect.size = _t_mask_size;
        _t_pre_imgV = [[UIImageView alloc] initWithFrame:infoRect];
        _t_pre_imgV.contentMode = UIViewContentModeScaleAspectFit;
        [self.view addSubview:_t_pre_imgV];
    }
    return _t_pre_imgV;
}

-(void)handelScanResponse:(NHScanResponse)response{
    _t_res_response = [response copy];
}

#pragma mark -- OpenCV Camera Delegate

- (void)processImage:(cv::Mat &)image {
    //NSLog(@"received opencv image ...");
    
    if (!_isScanning) {
        return;
    }
    
    /// set roi
    int sWidth = _t_mask_size.width;
    int sHeight = _t_mask_size.height;
    
    int m_width = image.size().width;
    int m_height = image.size().height;
    CGSize screenSize = [UIScreen mainScreen].bounds.size;
    CGFloat x_scale = screenSize.width/m_width;
    CGFloat y_scale = screenSize.height/m_height;
    cv::Rect destRect = cv::Rect((screenSize.width-sWidth)*0.5/x_scale,(screenSize.height-sHeight)*0.5/y_scale,sWidth/x_scale,sHeight/y_scale);
    Mat roi = Mat();
    image(destRect).copyTo(roi);
    
//    dispatch_async(dispatch_get_main_queue(), ^{
//        self.t_pre_imgV.image = [NHMatImgUtil UIImageFromCVMat:roi];
//    });
//    return;
    
    /// process mat
    if (NHScanTypeQR == _mScanType) {
        bool ret = qr_has_qr_code(image);
        //    NSLog(@"does img has qr:%d",ret);
        if (ret) {
            /// the image has a qr code than
            Mat tmp = Mat();
            //        int t_ret = qr_cutout(image, tmp);
            //        Mat dst = qr_get_qr_code(image);
            Mat dst = GetQRCode(roi);
            string info("");
            int t_ret = qr_scan_mat(dst, info);
            if (t_ret == 0) {
                [_camera stop];
                _isScanning = false;
                [self vibrate];
                NSString *t_info = [NSString stringWithUTF8String:info.c_str()];
                UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"扫描结果" message:t_info delegate:self cancelButtonTitle:@"确定" otherButtonTitles:nil, nil];
                dispatch_async(dispatch_get_main_queue(), ^{
                    [alert show];
                    //                self.t_pre_imgV.image = [NHMatImgUtil UIImageFromCVMat:dst];
                });
                cout << "recognised info:" << info << endl;
            }
        }
    }else if (NHScanTypeID == _mScanType) {
        Mat dst;
//        roi.convertTo(roi, CV_8U);
        int ret = ocr_getCard(roi, dst);
        if (ret == 0) {
            dst = ocr_getIDMat(dst);
            vector<Mat> vecRet;
            ret = ocr_segment(dst, vecRet);
//            cout << "segment size :" << vecRet.size() << endl;
            size_t numbers = vecRet.size();
            if (ret == 0 && numbers == 18) {
                
                cout << "识别结果: ";
                for (int i = 0 ;i < numbers; i++) {
                    int result = svm_recog(vecRet[i]);
                    cout << result ;
                }
                
                cout << endl;
                
//                UIImage *image = [NHMatImgUtil UIImageFromCVMat:vecRet[0]];
//                dispatch_async(dispatch_get_main_queue(), ^{
//                    self.t_pre_imgV.image = nil;
//                    self.t_pre_imgV.image = image;
//                });
            }
        }
        
    }else if (NHScanTypeBank == _mScanType) {
        Mat dst;
        //        roi.convertTo(roi, CV_8U);
        int ret = ocr_getCard(roi, dst);
        if (ret == 0) {
            dst = ocr_getBankMat(dst);
            UIImage *image = [NHMatImgUtil UIImageFromCVMat:dst];
            dispatch_async(dispatch_get_main_queue(), ^{
                self.t_pre_imgV.image = nil;
                self.t_pre_imgV.image = image;
            });
        }
    }
    
}

- (void)alertView:(UIAlertView *)alertView clickedButtonAtIndex:(NSInteger)buttonIndex{
//    [self.navigationController popViewControllerAnimated:true];
    if (_camera) {
        [_camera start];
        _isScanning = true;
    }
}


#pragma mark -- Verified Codes End --

#if !defined(__i386__) && defined(__ARM_NEON__)
#if  (!defined(__LP64__) && !defined(_LP64))
#define __MATH_NEON_32
#else
#define __MATH_NEON_64
#endif
#endif

/*
void
matvec3_RowMajor(float matrix[3][3], float v[3], float d[3])
{
    float *m = (float *) matrix;
    d[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2];
    d[1] = m[3]*v[0] + m[4]*v[1] + m[5]*v[2];
    d[2] = m[6]*v[0] + m[7]*v[1] + m[8]*v[2];
}

void __attribute__((noinline)) matvec3_neon_RowMajor(float m[3][3], float v[3], float d[3])
{
#if defined(__MATH_NEON_32)
    __asm__ volatile("lsr          %2, %2, #3      \n"
                     "# build the three constants: \n"
                     "mov         r4, #28          \n" // Blue channel multiplier
                     "mov         r5, #151         \n" // Green channel multiplier
                     "mov         r6, #77          \n" // Red channel multiplier
                     "vdup.8      d4, r4           \n"
                     "vdup.8      d5, r5           \n"
                     "vdup.8      d6, r6           \n"
                     "0:						   \n"
                     "# load 8 pixels:             \n"
                     "vld4.8      {d0-d3}, [%1]!   \n"
                     "# do the weight average:     \n"
                     "vmull.u8    q7, d0, d4       \n"
                     "vmlal.u8    q7, d1, d5       \n"
                     "vmlal.u8    q7, d2, d6       \n"
                     "# shift and store:           \n"
                     "vshrn.u16   d7, q7, #8       \n" // Divide q3 by 256 and store in the d7
                     "vst1.8      {d7}, [%0]!      \n"
                     "subs        %2, %2, #1       \n" // Decrement iteration count
                     "bne         0b            \n" // Repeat unil iteration count is not zero
                     :
                     : "r"(dest), "r"(src), "r"(numPixels)
                     : "r4", "r5", "r6"
                     );
#elif defined(__MATH_NEON_64)
    __asm__ volatile (
                      "ld1 {v3.2s}, [%1], 8 \n\t"    // V3 = v
                      "ld1 {v3.s}[2], [%1] \n\t"
                      
                      "ld3 {v0.2s, v1.2s, v2.2s}, [%0], 24 \n\t" // V0 = {x, m0_6, m0_3, m0_0}
                      "ld3 {v0.s, v1.s, v2.s}[2], [%0] \n\t"  // V1 = {x, m0_7, m0_4, m0_1}
                      // V2 = {x, m0_8, m0_5, m0_2}
                      
                      "fmul v9.4s, v0.4s, v3.s[0] \n\t"   // Multiply out
                      "fmla v9.4s, v1.4s, v3.s[1] \n\t"   //
                      "fmla v9.4s, v2.4s, v3.s[2] \n\t"   //
                      
                      "st1 {v9.2s}, [%2], 8 \n\t"    // Result in V9
                      "st1 {v9.s}[2], [%2] \n\t"    
                      
                      : "+r"(m), "+r"(v), "+r"(d)
                      :
                      : "v0", "v1", "v2", "v3", "v9", "memory"
                      );
#else
    matvec3_RowMajor(m, v, d);
#endif
}
 //*/

@end
