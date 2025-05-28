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
 * --sobel
 * --laplacian
 */

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

using namespace cv;

int main()
{
  Mat image;
  image = imread("./test.jpg");
  // vector of keyPoints
  std::vector<KeyPoint> keyPoints;
  Ptr<FeatureDetector> fast=FastFeatureDetector::create(40);
  // feature point detection
  fast->detect(image,keyPoints);
  drawKeypoints(image, keyPoints, image, Scalar::all(255), DrawMatchesFlags::DRAW_OVER_OUTIMG);
  imwrite("./demo24_fast_feature.jpg", image);
  return 0;
}
