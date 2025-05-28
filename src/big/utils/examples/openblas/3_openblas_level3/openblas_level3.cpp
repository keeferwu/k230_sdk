/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023/03/07     YeC          The first version
 *
 * @brief  Test for OpenBLAS Interface level3.
 * All the classes tested in this file are summarized as below:
 * --cblas_sgemm
 */

#include "cblas.h"
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

string expect = R"(7 10 15 22 )";

#define M 2
#define N 2
#define K 2
int main(int argc, char ** argv){
    float alpha=1;
    float beta=0;
    int lda=K;
    int ldb=N;
    int ldc=N;
    float A[M*K]={1,2,3,4};
    /* 1,2
     * 3,4
     */
    float B[K*N]={1,2,3,4};
    /* 1,2
     * 3,4
     */
    float C[M*N];
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, M, N, K, alpha, A, lda, B, ldb, beta, C, ldc);
    
    stringstream ss;
    streambuf   *buffer = cout.rdbuf();
    cout.rdbuf(ss.rdbuf());
    
    for(int i=0;i<M*N;i++)
        {cout<<C[i]<<" ";}
    
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

