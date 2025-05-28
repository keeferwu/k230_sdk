/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023/02/21     YeC          The first version
 *
 * @brief  Test for OpenCV/calcHist.
 * All the classes tested in this file are summarized as below:
 * --calcHist
 */

#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
using namespace std;
using namespace cv;
int main()
{
	Mat src = imread("./a.jpg");
	if (src.empty()) 
	{
		cout << "no picture" << endl;
		return -1;
	}
	//imshow("lena",src);
	vector<Mat> all_channel;
	split(src,all_channel); //split函数将图像的三通道分别提取出来，放到all_channel数组里面
	
	//定义参数变量
	const int bin = 256;
	float bin_range[2] = { 0,255 };
	const float* ranges[1] = { bin_range };//这样做只是方便下面clacHist函数的传参
	
	//定义变量来存储直方图数据
	Mat b_hist;
	Mat g_hist;
	Mat r_hist;
	
	//计算得到直方图数据
	calcHist(&all_channel[0], 1, 0, Mat(), b_hist, 1, &bin, ranges, true, false);
	calcHist(&all_channel[1], 1, 0, Mat(), g_hist, 1, &bin, ranges, true, false);
	calcHist(&all_channel[2], 1, 0, Mat(), r_hist, 1, &bin, ranges, true, false);

	//设置直方图画布的参数
	int hist_w = 512;  
	int hist_h = 400;
	int bin_w = cvRound((double)hist_w/bin); //设置直方图中每一点的步长，通过hist_w/bin计算得出。cvRound()函数是“四舍五入”的作用。
	Mat hist_canvas = Mat::zeros(hist_h, hist_w, CV_8UC3);
	//直方图数据进行归一化。
	normalize(b_hist, b_hist, 0, 255, NORM_MINMAX, -1, Mat());
	normalize(g_hist, g_hist, 0, 255, NORM_MINMAX, -1, Mat());
	normalize(r_hist, r_hist, 0, 255, NORM_MINMAX, -1, Mat());
	//统计图绘制
	Mat hist;
	double max_val;
	minMaxLoc(hist, 0, &max_val, 0, 0);//计算直方图的最大像素值
	for (int i = 1; i < 256; i++)
	{
		//绘制蓝色分量直方图
		line(hist_canvas, Point((i - 1)*bin_w, hist_h - cvRound(b_hist.at<float>(i - 1))),
			Point((i)*bin_w, hist_h - cvRound(b_hist.at<float>(i))), Scalar(255, 0, 0),2);
		//绘制绿色分量直方图
		line(hist_canvas, Point((i - 1)*bin_w, hist_h - cvRound(g_hist.at<float>(i - 1))),
			Point((i)*bin_w, hist_h - cvRound(g_hist.at<float>(i))), Scalar(0, 255, 0),2);
		//绘制红色分量直方图
		line(hist_canvas, Point((i - 1)*bin_w, hist_h - cvRound(r_hist.at<float>(i - 1))),
			Point((i)*bin_w, hist_h - cvRound(r_hist.at<float>(i))), Scalar(0, 0, 255),2);
	}
	//展示到屏幕上
	imwrite("lena_hist.jpg", hist_canvas);
	//imshow("result",hist_canvas);
	//waitKey(0);
	//destroyAllWindows();
	return 0;
}

