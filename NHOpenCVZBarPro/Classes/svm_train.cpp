//
//  svm_train.cpp
//  NHSVMTrainPro
//
//  Created by hu jiaju on 15/10/26.
//  Copyright © 2015年 hu jiaju. All rights reserved.
//

#include "svm_train.hpp"

#define HORIZONTAL    1
#define VERTICAL      0
#define TrainItems    550

CvANN_MLP *ann;
CvSVM *svm;
KNearest *knn;


int svm_init() {
    svm = new CvSVM();
    ann = new CvANN_MLP();
    
    knn = new CvKNearest();
    return 0;
}

int svm_save() {
    
    svm->save("/Users/nanhujiaju/Desktop/train.xml");
    //    delete svm;
    return 0;
}

Mat ProjectedHistogram(Mat img, int t) {
    int sz=(t)?img.rows:img.cols;
    Mat mhist=Mat::zeros(1,sz,CV_32F);
    
    for(int j=0; j<sz; j++){
        Mat data=(t)?img.row(j):img.col(j);
        mhist.at<float>(j)=countNonZero(data);
    }
    
    //Normalize histogram
    double min, max;
    minMaxLoc(mhist, &min, &max);
    
    if(max>0)
        mhist.convertTo(mhist,-1 , 1.0f/max, 0);
    
    return mhist;
}

Mat features(Mat in, int sizeData){
    //Histogram features
    Mat vhist=ProjectedHistogram(in,VERTICAL);
    Mat hhist=ProjectedHistogram(in,HORIZONTAL);
    
    //Low data feature
    Mat lowData;
    resize(in, lowData, Size(sizeData, sizeData) );
    
    //Last 10 is the number of moments components
    int numCols=vhist.cols+hhist.cols+lowData.cols*lowData.cols;
    
    Mat out=Mat::zeros(1,numCols,CV_32F);
    //Asign values to feature
    int j=0;
    for(int i=0; i<vhist.cols; i++)
    {
        out.at<float>(j)=vhist.at<float>(i);
        j++;
    }
    for(int i=0; i<hhist.cols; i++)
    {
        out.at<float>(j)=hhist.at<float>(i);
        j++;
    }
    for(int x=0; x<lowData.cols; x++)
    {
        for(int y=0; y<lowData.rows; y++){
            out.at<float>(j)=(float)lowData.at<unsigned char>(x,y);
            j++;
        }
    }
    
    return out;
}

Mat features2(Mat in, int sizeData)
{
    //Histogram features
    
    Mat vhist = ProjectedHistogram(in, VERTICAL);
    Mat hhist = ProjectedHistogram(in, HORIZONTAL);
    //Low data feature
    Mat lowData;
    resize(in, lowData, Size(sizeData, sizeData));
    
    
    //Last 10 is the number of moments components
    int numCols = vhist.cols + hhist.cols + lowData.cols*lowData.cols;
    //int numCols = vhist.cols + hhist.cols;
    Mat out = Mat::zeros(1, numCols, CV_32F);
    //Asign values to feature,ANN的样本特征为水平、垂直直方图和低分辨率图像所组成的矢量
    int j = 0;
    for (int i = 0; i<vhist.cols; i++)
    {
        out.at<float>(j) = vhist.at<float>(i);
        j++;
    }
    for (int i = 0; i<hhist.cols; i++)
    {
        out.at<float>(j) = hhist.at<float>(i);
        j++;
    }
    for (int x = 0; x<lowData.cols; x++)
    {
        for (int y = 0; y<lowData.rows; y++) {
            out.at<float>(j) = (float)lowData.at<unsigned char>(x, y);
            j++;
        }
    }
    //if(DEBUG)
    //  cout << out << "\n===========================================\n";
    return out;
}

#define NHCharSize                  20
#define NUMBER_OF_CLASSES           11
#define ATTRIBUTES_PER_SAMPLE       50
#define NUMBER_OF_TRAINING_SAMPLES  (NUMBER_OF_CLASSES*ATTRIBUTES_PER_SAMPLE)
int svm_trainForPath(vector<vector<string>> paths) {
    
    svm_init();
    
    size_t size = paths.size();
    cout << "size : " << size << endl;
    
    CvSVMParams params;
    params.svm_type = CvSVM::C_SVC;
    params.kernel_type = CvSVM::LINEAR;
    params.degree = 0.1;
    params.gamma = 1;
    params.coef0 = 0.1;
    params.C = 1;
    params.nu = 0.1;
    params.p = 0.1;
    params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 1000, 0.0001);
    
    // will hold training data
    Mat trainingMat(NUMBER_OF_TRAINING_SAMPLES, NHCharSize*NHCharSize, CV_32FC1);
    // hold labels -> training filenames
    vector<int> labels;
    
    int indexs = 0;
    for (int index = 0 ; index < NUMBER_OF_CLASSES; index++) {
        
        vector<string> inners = paths[index];
        size_t inner_size = inners.size();
        for (int k = 0; k < inner_size; k++) {
            
            Mat img = imread(inners[k].c_str(),IMREAD_GRAYSCALE);
            if (img.empty()) {
                continue;
            }
            resize(img,img,Size(NHCharSize,NHCharSize));
            
            int ii = 0;
            for (int i = 0; i<img.rows; i++) {
                for (int j = 0; j < img.cols; j++) {
                    trainingMat.at<float>(indexs, ii++) = img.at<uchar>(i,j);
                }
            }
            indexs++;
            labels.push_back(index);
        }
    }
    
    int labelsArray[labels.size()];
    
    // loop over labels
    for(int index=0; index<labels.size(); index++){
        labelsArray[index] = labels[index];
    }
    
    Mat labelsMat((int)labels.size(), 1, CV_32S, labelsArray);
    
    svm->train(trainingMat, labelsMat,Mat(),Mat(),params);
//    svm_save();
    
    return 0;
}

float svm_recog(Mat features){
    float result = -1;
    if (features.empty()) {
        return result;
    }
    Mat img = features;
    resize(img,img,CvSize(NHCharSize,NHCharSize));
    threshold(img,img,125,255,THRESH_BINARY);
    Mat dst = img.clone().reshape(1,1);/// lower dim 2D 2 1D
    dst.convertTo(dst, CV_32FC(1));
//     cvNot(&img, &img);/// IPlimage type
//     bitwise_not(img, img); /// mat type
    
    result = svm->predict(dst);
    
    return result;
}

