//
//  ocr_card.hpp
//  NHOpenCVZBarPro
//
//  Created by hu jiaju on 15/10/29.
//  Copyright © 2015年 hu jiaju. All rights reserved.
//

#ifndef ocr_card_hpp
#define ocr_card_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>

#endif /* ocr_card_hpp */

using namespace cv;
using namespace std;


int ocr_getCard(Mat src, Mat &card);

Mat ocr_getIDMat(Mat src);

Mat ocr_getBankMat(Mat src);

int ocr_segment(Mat src, vector<Mat> & chars);
