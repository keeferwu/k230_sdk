/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023/03/07     YeC          The first version
 *
 * @brief  Test for OpenBLAS fortran interface.
 * All the classes tested in this file are summarized as below:
 * --openblas fortran inference
 */

#include "stdio.h"
#include "stdlib.h"
#include "f77blas.h"
#include "sys/time.h"
#include "time.h"
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

extern void dgemm_(char*, char*, int*, int*,int*, double*, double*, int*, double*, int*, double*, double*, int*);

string expect = R"(16.801 18.002 18.003 16.801 15.602 22.803 )";

int main(int argc, char* argv[])
{
  int i = 0;
  int m = 2;
  int n = 3;
  int k = 4;
  int sizeofa = m * k;
  int sizeofb = k * n;
  int sizeofc = m * n;
  char ta = 'N';
  char tb = 'N';
  double alpha = 1.2;
  double beta = 0.001;

  double* A = (double*)malloc(sizeof(double) * sizeofa);
  double* B = (double*)malloc(sizeof(double) * sizeofb);
  double* C = (double*)malloc(sizeof(double) * sizeofc);

  srand((unsigned)time(NULL));

  for (i=0; i<sizeofa; i++)
    A[i] = i%3+1;//(rand()%100)/10.0;

  for (i=0; i<sizeofb; i++)
    B[i] = i%3+1;//(rand()%100)/10.0;

  for (i=0; i<sizeofc; i++)
    C[i] = i%3+1;//(rand()%100)/10.0;

  printf("m=%d,n=%d,k=%d,alpha=%lf,beta=%lf,sizeofc=%d\n",m,n,k,alpha,beta,sizeofc);
  dgemm_(&ta, &tb, &m, &n, &k, &alpha, A, &m, B, &k, &beta, C, &m);

  printf("This is matrix A\n\n");
  for(i=0; i < sizeofa; i++)
    printf("%lf ", A[i]);
  printf("\n");
  
  printf("This is matrix B\n\n");
  for(i=0; i < sizeofb; i++)
    printf("%lf ", B[i]);
  printf("\n");
  
  stringstream ss;
  streambuf   *buffer = cout.rdbuf();
  cout.rdbuf(ss.rdbuf());
  
  for(int i=0;i<sizeofc;i++)
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

  free(A);
  free(B);
  free(C);
  return 0;
}
