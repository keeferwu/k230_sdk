/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023/03/07     YeC          The first version
 *
 * @brief  Test for OpenBLAS Interface level2.
 * All the classes tested in this file are summarized as below:
 * --cblas_saxpy
 */

#include "cblas.h"
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

string expect = R"(20 40 10 20 30 60 )";

int main(int argc, char ** argv)
{
    float x[2] = {1.0, 2.0};
    float y[3] = {2.0, 1.0, 3.0};
    float A[6] = { 0 };
    blasint rows = 2, cols = 3;
    float alpha = 10;
    blasint inc_x = 1, inc_y = 1;
    blasint lda = 2;

    //矩阵按列优先存储
    //A <== alpha*x*y' + A （y'表示y的转置）
    cblas_sger(CblasColMajor, rows, cols, alpha, x, inc_x, y, inc_y, A, lda);
    
    
    stringstream ss;
    streambuf   *buffer = cout.rdbuf();
    cout.rdbuf(ss.rdbuf());
  
    for(int i=0;i<rows;i++)
    {
    	for(int j=0;j<cols;j++)
    	{
           cout<<A[i*cols+j]<<" ";
    	}
    }
  
    cout.rdbuf(buffer);
    string s(ss.str());
    cout << "*********************************************************" << endl;
    cout << "This is the result:" << endl;
    cout << s << endl;
    cout << "*********************************************************" << endl;
    cout << "This is the reference:" << endl;
    cout << expect << endl;

    if (expect == s)
      cout << "{Test PASS}." << endl;
    else
      cout << "{Test FAIL}." << endl;
    
    return 0;
}

