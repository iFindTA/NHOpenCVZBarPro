//
//  svm_train.hpp
//  NHSVMTrainPro
//
//  Created by hu jiaju on 15/10/26.
//  Copyright © 2015年 hu jiaju. All rights reserved.
//

#ifndef svm_train_hpp
#define svm_train_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;

int svm_trainForPath(vector<vector<string>> paths);

float svm_recog(Mat features);

#endif /* svm_train_hpp */
