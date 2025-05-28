/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023/03/07     YeC          The first version
 *
 * @brief  Test for OpenBLAS level1 interface.
 * All the classes tested in this file are summarized as below:
 * --cblas_saxpy
 */

#include "cblas.h"
#include <iostream>
#include <string>
#include <sstream>
#define N 4

using namespace std;

string expect = R"(4 7 11 14 )";

int main(int argc, char ** argv)
{
    float alpha=3;
    float x[4]={1.0,2,3,4};
    /* 1,2,3,4
     */
    float y[4]={1,1,2,2};
    /* 1,1,2,2
     */

    cblas_saxpy(N, alpha, x , 1, y, 1);

    stringstream ss;
    streambuf   *buffer = cout.rdbuf();
    cout.rdbuf(ss.rdbuf());
  
    for(int i=0;i<N;i++)
      {cout<<y[i]<<" ";}
  
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

