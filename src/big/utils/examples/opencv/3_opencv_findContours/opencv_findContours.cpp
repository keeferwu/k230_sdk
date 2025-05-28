/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023/02/21     YeC          The first version
 *
 * @brief  Test for OpenCV/findContours.
 * All the classes tested in this file are summarized as below:
 * --findContours
 */

#include <iostream>
#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <stdlib.h>
 
using namespace std;
using namespace cv;
 
int main()
{
    Mat src, grayImage, dstImage;
    src = imread("./a.jpg");
 
    //判断图像是否加载成功
    if (src.empty())
    {
        cout << "图像加载失败" << endl;
        return -1;
    }
 
    //imshow("lena", src);
 
    //转换为灰度图并平滑滤波
    cvtColor(src, grayImage, COLOR_BGR2GRAY);
 
    //定义变量
    vector<vector<Point>>contours;
    vector<Vec4i>hierarchy;
 
    grayImage = grayImage > 120;
    findContours(grayImage, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
 
    //绘制轮廓图
    dstImage = Mat::zeros(grayImage.size(), CV_8UC3);
    for (long unsigned int i = 0; i < hierarchy.size(); i++)
    {
        Scalar color = Scalar(rand() % 255, rand() % 255, rand() % 255);
        drawContours(dstImage, contours, i, color, CV_FILLED, 8, hierarchy);
    }
    imwrite("lena_contours.jpg", dstImage);
    //imshow("lena_contours", dstImage);
    //waitKey(0);
    //destroyAllWindows();
    return 0;
}

