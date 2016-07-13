//
//  ocr_card.cpp
//  NHOpenCVZBarPro
//
//  Created by hu jiaju on 15/10/29.
//  Copyright © 2015年 hu jiaju. All rights reserved.
//

#define CARD_MIN_AREA        200
#define CARD_MAX_AREA        400
#define M_PI                 3.1415926535897

#include "ocr_card.hpp"

typedef struct {
    CvRect top;
    CvRect bottom;
    CvRect left;
    CvRect right;
} DetectionBoxes;

const float DEFAULT_ERROR = 0.6;//0.6
const float DEFAULT_ASPECT = 3.75; //3.75
//! verifySize所用常量
static const int DEFAULT_VERIFY_MIN = 1;//3
static const int DEFAULT_VERIFY_MAX = 24;//20
static const int CHAR_SIZE = 20;
static const int HORIZONTAL = 1;
static const int VERTICAL = 0;

double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 ) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

//! 对minAreaRect获得的最小外接矩形，用纵横比进行判断
bool verifySizes(RotatedRect mr)
{
    float error = DEFAULT_ERROR;
    //Spain car plate size: 52x11 aspect 4,7272
    //China car plate size: 440mm*140mm，aspect 3.142857
    
    //Real car plate size: 136 * 32, aspect 4
    float aspect = DEFAULT_ASPECT;
    
    //Set a min and max area. All other patchs are discarded
    //int min= 1*aspect*1; // minimum area
    //int max= 2000*aspect*2000; // maximum area
    int min = 34 * 8 * DEFAULT_VERIFY_MIN; // minimum area
    int max = 34 * 8 * DEFAULT_VERIFY_MAX; // maximum area
    
    //Get only patchs that match to a respect ratio.
    float rmin= aspect-aspect*error;
    float rmax= aspect+aspect*error;
    
    int area= mr.size.height * mr.size.width;
    float r = (float)mr.size.width / (float)mr.size.height;
    if(r < 1)
        r= (float)mr.size.height / (float)mr.size.width;
    
    if(( area < min || area > max ) || ( r < rmin || r > rmax ))
        return false;
    else
        return true;
}

//! 字符尺寸验证
bool verifyCharSizes(Mat r){
    //Char sizes 45x90
    float aspect = 45.0f / 90.0f;
    float charAspect = (float)r.cols / (float)r.rows;
    float error = 0.7;
    float minHeight = 10;
    float maxHeight = 35;
    //We have a different aspect ratio for number 1, and it can be ~0.2
    float minAspect = 0.05;
    float maxAspect = aspect + aspect*error;
    //area of pixels
    float area = countNonZero(r);
    //bb area
    float bbArea = r.cols*r.rows;
    //% of pixel in area
    float percPixels = area / bbArea;
    
    if (percPixels <= 1 && charAspect > minAspect && charAspect < maxAspect && r.rows >= minHeight && r.rows < maxHeight)
        return true;
    else
        return false;
}

//! 字符预处理
Mat preprocessChar(Mat in){
    //Remap image
    int h = in.rows;
    int w = in.cols;
    int charSize = CHAR_SIZE;	//统一每个字符的大小
    Mat transformMat = Mat::eye(2, 3, CV_32F);
    int m = max(w, h);
    transformMat.at<float>(0, 2) = m / 2 - w / 2;
    transformMat.at<float>(1, 2) = m / 2 - h / 2;
    
    Mat warpImage(m, m, in.type());
    warpAffine(in, warpImage, transformMat, warpImage.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(0));
    
    Mat out;
    resize(warpImage, out, Size(charSize, charSize));
    
    return out;
}

//! 将Rect按位置从左到右进行排序
int SortRect(const vector<Rect>& vecRect, vector<Rect>& out)
{
    vector<int> orderIndex;
    vector<int> xpositions;
    
    for (int i = 0; i < vecRect.size(); i++)
    {
        orderIndex.push_back(i);
        xpositions.push_back(vecRect[i].x);
    }
    
    float min = xpositions[0];
    int minIdx = 0;
    for (int i = 0; i< xpositions.size(); i++)
    {
        min = xpositions[i];
        minIdx = i;
        for (int j = i; j<xpositions.size(); j++)
        {
            if (xpositions[j]<min){
                min = xpositions[j];
                minIdx = j;
            }
        }
        int aux_i = orderIndex[i];
        int aux_min = orderIndex[minIdx];
        orderIndex[i] = aux_min;
        orderIndex[minIdx] = aux_i;
        
        float aux_xi = xpositions[i];
        float aux_xmin = xpositions[minIdx];
        xpositions[i] = aux_xmin;
        xpositions[minIdx] = aux_xi;
    }
    
    for (int i = 0; i<orderIndex.size(); i++)
    {
        out.push_back(vecRect[orderIndex[i]]);
    }
    
    return 0;
}

Point2f computeIntersect(Vec4i a, Vec4i b) {
    int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3];
    int x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];
    
    if (float d = ((float)(x1-x2) * (y3-y4)) - ((y1-y2) * (x3-x4)))
    {
        Point2f pt;
        pt.x = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)) / d;
        pt.y = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)) / d;
        return pt;
    }
    else
        return Point2f(-1, -1);
}

void sortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center)
{
    std::vector<cv::Point2f> top, bot;
    
    for (int i = 0; i < corners.size(); i++)
    {
        if (corners[i].y < center.y)
            top.push_back(corners[i]);
        else
            bot.push_back(corners[i]);
    }
    
    cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
    cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
    cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
    cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];
    
    corners.clear();
    corners.push_back(tl);
    corners.push_back(tr);
    corners.push_back(br);
    corners.push_back(bl);
}

void find_squares(Mat& image, vector<vector<Point> >& squares)
{
    // blur will enhance edge detection
    Mat blurred(image);
    medianBlur(image, blurred, 5);
    
    Mat gray0(blurred.size(), CV_8U), gray;
    vector<vector<Point> > contours;
    
    // find squares in every color plane of the image
    for (int c = 0; c < 3; c++)
    {
        int ch[] = {c, 0};
        mixChannels(&blurred, 1, &gray0, 1, ch, 1);
        
        // try several threshold levels
        const int threshold_level = 2;
        for (int l = 0; l < threshold_level; l++)
        {
            // Use Canny instead of zero threshold level!
            // Canny helps to catch squares with gradient shading
            if (l == 0)
            {
                Canny(gray0, gray, 10, 20, 3); //
                
                // Dilate helps to remove potential holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                gray = gray0 >= (l+1) * 255 / threshold_level;
            }
            
            // Find contours and store them in a list
            findContours(gray, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            
            // Test contours
            vector<Point> approx;
            for (size_t i = 0; i < contours.size(); i++)
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
                
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if (approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    isContourConvex(Mat(approx)))
                {
                    double maxCosine = 0;
                    
                    for (int j = 2; j < 5; j++)
                    {
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }
                    
                    if (maxCosine < 0.3)
                        squares.push_back(approx);
                }
            }
        }
    }
}

int ocr_getCard(Mat src, Mat &card) {
    if (src.empty()) {
        return -1;
    }
    
    //*
    src.copyTo(card);
    vector< vector <Point> > contours;
    find_squares(src, contours);
//    drawContours(src, contours, -1, Scalar(0,255,0),2);
    
//    cout << "point count :" << contours.size() << endl;
    int largest_contour_index=0;
    int largest_area=0;
    size_t counts = contours.size();
    if (counts <= 0) {
        return -1;
    }
    for( int i = 0; i< contours.size(); i++ ){
        double a = contourArea( contours[i],false);  //  Find the area of contour
        if( a > largest_area ){
//            cout << "find rect area : " << a << endl;
            largest_area=a;
            largest_contour_index=i;//Store the index of largest contour
        }
    }
    vector<vector<Point2f> > contours_poly(1);
    approxPolyDP( Mat(contours[largest_contour_index]), contours_poly[0],5, true );
    if(contours_poly[0].size()>=4){
        // Get mass center
        Point2f center(0,0);
        for (int i = 0; i < contours_poly[0].size(); i++){
            center.x += contours_poly[0][i].x;
            center.y += contours_poly[0][i].y;
        }
        
        center.x *= (1. / contours_poly[0].size());
        center.y *= (1. / contours_poly[0].size());
        sortCorners(contours_poly[0], center);
        
        std::vector<Point2f> quad_pts;
        std::vector<Point2f> squre_pts;
        quad_pts.push_back(Point2f(contours_poly[0][0].x,contours_poly[0][0].y));
        quad_pts.push_back(Point2f(contours_poly[0][1].x,contours_poly[0][1].y));
        quad_pts.push_back(Point2f(contours_poly[0][2].x,contours_poly[0][2].y));
        quad_pts.push_back(Point2f(contours_poly[0][3].x,contours_poly[0][3].y));
        
//        squre_pts.push_back(Point2f(boundRect.x,boundRect.y));
//        squre_pts.push_back(Point2f(boundRect.x,boundRect.y+boundRect.height));
//        squre_pts.push_back(Point2f(boundRect.x+boundRect.width,boundRect.y+boundRect.height));
//        squre_pts.push_back(Point2f(boundRect.x+boundRect.width,boundRect.y));
        
        squre_pts.push_back(Point2f(0,0));
        squre_pts.push_back(Point2f(src.size().width,0));
        squre_pts.push_back(Point2f(src.size().width,src.size().height));
        squre_pts.push_back(Point2f(0,src.size().height));
        
        Mat transmtx = getPerspectiveTransform(quad_pts,squre_pts);
        card = Mat::zeros(src.rows, src.cols, CV_8UC1);
        warpPerspective(src, card, transmtx, src.size());
        
        
//        squre_pts.push_back(Point2f(0,0));
//        squre_pts.push_back(Point2f(0,src.size().height));
//        squre_pts.push_back(Point2f(src.size().width,src.size().height));
//        Mat affineTrans = getAffineTransform(quad_pts, squre_pts);
//        Mat warped;
//        warpAffine(src, warped, affineTrans, src.size());
//        warped.copyTo(card);
        
        return 0;
    }
    
    return -1;
    //*/
    
    ////----------------------------------------/////
    
    /*
    Mat gray(src.size(),CV_8U);
    cvtColor(src,gray,COLOR_BGR2GRAY);
    threshold( gray, gray, 120, 255,THRESH_BINARY);
//    adaptiveThreshold(thr, thr, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 25, 10);
//    medianBlur(thr, thr, 5);
//    GaussianBlur(thr, thr, Size(1,1), 1000);
    
    vector< vector <Point> > contours; // Vector for storing contour
    vector< Vec4i > hierarchy;
    int largest_contour_index=0;
    int largest_area=0;
    double maxArea = (src.rows-3)*(src.cols-3);
    cout << "max area :" << maxArea << endl;
    findContours( gray, contours, hierarchy,RETR_EXTERNAL, CHAIN_APPROX_NONE ); // Find the contours in the image
    for( int i = 0; i< contours.size(); i++ ){
        
        double a = contourArea( contours[i],false);  //  Find the area of contour
        if( a > largest_area && a < maxArea){
            cout << "find rect area : " << a << endl;
            largest_area=a;
            largest_contour_index=i;//Store the index of largest contour
        }
    }
    
    drawContours( card,contours, largest_contour_index, Scalar(255,255,255),CV_FILLED, 8, hierarchy );
    vector<vector<Point> > contours_poly(1);
    approxPolyDP( Mat(contours[largest_contour_index]), contours_poly[0],5, true );
    if(contours_poly[0].size()==4){
        std::vector<Point2f> quad_pts;
        std::vector<Point2f> squre_pts;
        quad_pts.push_back(Point2f(contours_poly[0][0].x,contours_poly[0][0].y));
        quad_pts.push_back(Point2f(contours_poly[0][1].x,contours_poly[0][1].y));
        quad_pts.push_back(Point2f(contours_poly[0][2].x,contours_poly[0][2].y));
        quad_pts.push_back(Point2f(contours_poly[0][3].x,contours_poly[0][3].y));
        squre_pts.push_back(Point2f(0,0));
        squre_pts.push_back(Point2f(0,src.size().height));
        squre_pts.push_back(Point2f(src.size().width,src.size().height));
        squre_pts.push_back(Point2f(src.size().width,0));
        
        Mat transmtx = getPerspectiveTransform(quad_pts,squre_pts);
        Mat transformed = Mat::zeros(src.rows, src.cols, CV_8UC3);
        warpPerspective(src, transformed, transmtx, src.size());
        
        card = transformed;
    }else{
        cout<<"Make sure that your are getting 4 corner using approxPolyDP..."<<endl;
        return -1;
    }
    return 0;
    //*/
    
}

// 以下定义宏为从有效身份证区域截取 姓名&地址 和 号码 子区域 用
#define NAMEROI_WIDTH 0.48
#define NAMEROI_HEIGH 0.75
#define NAMEROI_XPOS  0.175
#define NAMEROI_YPOS  0.05

#define ID_WIDTH 0.60
#define ID_HEIGH 0.15
#define ID_XPOS  0.30
#define ID_YPOS  0.77

#define BANK_WIDTH 0.90
#define BANK_HEIGH 0.13
#define BANK_XPOS  0.05
#define BANK_YPOS  0.55

// 以下定义宏为从 姓名&地址 和 号码 子区域 进行二值分割用

#define NAME_THRE       15
#define ID_THRE         30
#define BANK_THRE       10

Mat ocr_getIDMat(Mat src) {
    if (src.empty()) {
        return Mat();
    }
    
    Mat dst ;
    src(Rect(src.cols*ID_XPOS,src.rows*ID_YPOS,src.cols*ID_WIDTH,src.rows*ID_HEIGH)).copyTo(dst);
    ///gray
    Mat gray(dst.size(),CV_8U);
    cvtColor(dst,gray,COLOR_BGR2GRAY);
//    threshold( gray, gray, 120, 255,THRESH_BINARY);
    adaptiveThreshold(gray, gray, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 25, 10);
    medianBlur(gray, gray, 5);
    return gray;
}

#pragma mark == bank card ==

//! 直方图均衡
Mat histeq(Mat in)
{
    Mat out(in.size(), in.type());
    if (in.channels() == 3)
    {
        Mat hsv;
        vector<Mat> hsvSplit;
        cvtColor(in, hsv, COLOR_BGR2HSV);
        split(hsv, hsvSplit);
        equalizeHist(hsvSplit[2], hsvSplit[2]);
        merge(hsvSplit, hsv);
        cvtColor(hsv, out, COLOR_HSV2BGR);
    }
    else if (in.channels() == 1)
    {
        equalizeHist(in, out);
    }
    return out;
}

// 将BGR颜色归一化，消除线性光照影响
bool colorNormal(Mat& img)
{
    if (img.channels() != 3)
    {
        return false;
    }
    
    int nl= img.rows; // number of lines
    int nc= img.cols ; // number of columns
    // is it a continous image?
    if (img.isContinuous())
    {
        // then no padded pixels
        nc= nc*nl;
        nl= 1;  // it is now a 1D array
    }
    
    double bgrSum;
    // for all pixels
    for (int j=0; j<nl; j++)
    {
        // pointer to first column of line j
        uchar* data= img.ptr(j);
        for (int i=0; i<nc; i++)
        {
            // process each pixel --------
            bgrSum= 255.0/(1 +data[0] +data[1] +data[2]);
            data[0] *=bgrSum;
            data[1] *=bgrSum;
            data[2] *=bgrSum;
            data +=3;
        }
    }
    
    return true;
    
}

Mat ocr_getBankMat(Mat src) {
    if (src.empty()) {
        return Mat();
    }
    
    Mat dst ;
    src(Rect(src.cols*BANK_XPOS,src.rows*BANK_YPOS,src.cols*BANK_WIDTH,src.rows*BANK_HEIGH)).copyTo(dst);

    ///颜色归一化 效果不佳
//    cvtColor(dst, dst, COLOR_BGRA2BGR);
//    bool ret = colorNormal(dst);
//    cout << "normal ret :" << ret << "channel" << dst.channels() << endl;
    
    
    ///gray
    Mat gray(dst.size(),CV_8U);
    cvtColor(dst,gray,COLOR_BGR2GRAY);
    
    // 为了消除光照影响，对裁剪图像使用直方图均衡化处理
    blur(gray, gray, Size(3, 3));
    gray = histeq(gray);
    
    int g_nStructElementSize = 3; //结构元素(内核矩阵)的尺寸
    //获取自定义核
    Mat element = getStructuringElement(MORPH_RECT,
                                        Size(2*g_nStructElementSize+1,2*g_nStructElementSize+1),
                                        Point( g_nStructElementSize, g_nStructElementSize ));
//    Mat tmp;
    erode(gray, gray, element);
    
    dilate(gray, gray, element);
//    gray = tmp - gray;
    
//    GaussianBlur(gray, gray, Size(1,1), 1000);
//    threshold( gray, gray, 120, 255,THRESH_BINARY);
//    adaptiveThreshold(gray, gray, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 25, 10);
//    medianBlur(gray, gray, 5);
    return gray;
}

int ocr_segment(Mat src, vector<Mat> & chars) {
    if (src.empty()) {
        return -1;
    }
    
    Mat img_contours;
    src.copyTo(img_contours);
    
    vector< vector< Point> > contours;
    findContours(img_contours,
                 contours, // a vector of contours
                 RETR_EXTERNAL, // retrieve the external contours
                 CHAIN_APPROX_NONE); // all pixels of each contours
    
    //Start to iterate to each contour founded
    vector<vector<Point> >::iterator itc = contours.begin();
    vector<Rect> vecRect;
    
    //Remove patch that are no inside limits of aspect ratio and area.
    //将不符合特定尺寸的图块排除出去
    while (itc != contours.end())
    {
        cv::Rect mr = boundingRect(Mat(*itc));
        Mat auxRoi(src, mr);
        if (verifyCharSizes(auxRoi))
            vecRect.push_back(mr);
        
        ++itc;
    }
    
    if (vecRect.size() == 0)
        return -3;
    
    vector<Rect> sortedRect;
    ////对符合尺寸的图块按照从左到右进行排序
    SortRect(vecRect, sortedRect);
    
    for (int i = 0; i < sortedRect.size(); i++) {
        Rect mr = sortedRect[i];
        Mat auxRoi(src, mr);
        
        auxRoi = preprocessChar(auxRoi);
        chars.push_back(auxRoi);
    }
    
    return 0;
}