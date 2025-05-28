/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022/11/04     YeC          The first version
 *
 * @brief  Test for OpenCV/Module/Core/mat.hpp.
 * All the functions tested in this file are summarized as below:
 * --threshold
 */

#include<opencv2/opencv.hpp>
#include<iostream>

using namespace cv;
using namespace std;

int main()
{
    Mat lena = imread("./1.bmp", 1);
    if (!lena.data) {
        cout << "Input Image reading error !" << endl;
		return -1;
    }

    Mat lena_gray, lena_threshold;
  	cvtColor(lena, lena_gray, COLOR_BGR2GRAY);
    imwrite("./lena_gray.jpg", lena_gray);

    threshold(lena_gray, lena_threshold, 170, 255, THRESH_BINARY);

    imwrite("./lena_threshold.jpg", lena_threshold);

    return 0;
}
