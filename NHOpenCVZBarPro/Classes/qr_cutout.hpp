//
//  qr_cutout.hpp
//  NHOpenCVPro
//
//  Created by hu jiaju on 15/10/12.
//  Copyright © 2015年 hu jiaju. All rights reserved.
//

#ifndef qr_cutout_hpp
#define qr_cutout_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>
#include <cmath>

#endif /* qr_cutout_hpp */

using namespace cv;
using namespace std;

Mat qr_crop(Mat src, cv::Rect rect);

bool qr_has_qr_code(Mat src);

int qr_cutout(Mat origin,Mat dstMat);

Mat qr_get_code_test(string file);
Mat qr_get_qr_code(Mat src);

Mat GetQRCode (Mat &image);

int qr_scan_mat(Mat src, string &info);