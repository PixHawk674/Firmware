//
//  StateEstimator.h
//  
//
//  Created by Timothy Bright on 12/2/14.
//
//

#ifndef _StateEstimator_h
#define _StateEstimator_h

#define GRAVITY 9.81

class StateEstimator
{
public:
    static void Estimate(double* xhat, double* uu, double* P);
    static double* P1;
    static double* Q1;
    static double* R1;
    
    static double* P2;
    static double* Q2;
    static double* R2;
    
    
private:
    static double _pHat;
    static double _qHat;
    static double _rHat;
    //static double _staticPresHat = 0;
    //static double _diffPresHat = 0;
    static double _hHat;
    static double _Vahat;
    static double _phiHat;
    static double _thetaHat;
    static double _pnHat;
    static double _peHat;
    static double _chiHat;
    static double _VgHat;
    static double _psiHat;
    static double _wnHat;
    static double _weHat;
    static double _alphaHat;
    static double _betaHat;
    static double _bxHat;
    static double _byHat;
    static double _bzHat;
    
    static double LPF(double u, double y, double alpha);
    static void RollPitchEstimator(double* xhat, double* x0, double* u, double Ts, double* P, double* Q, double* R);
    static void CoursePositionEstimator(double* xhat, double* x0, double* u, double Ts, double* P, double* Q, double* R);
};

class Matrix
{
public:
    static void Multiply(double* c, double* a, double* b, int m, int n, int p)
    {
        // a[m x n] * b[n x p] = c[m x p]
        for(int i = 0; i < m; i++)
        {
            for(int j = 0; j < p; j++)
            {
                double val = 0;
                for(int k = 0; k < n; k++)
                {
                    val += a[i * n + k] * b[k * p + j];
                }
                c[i * n + j] = val;
            }
        }
    }
    static void Transpose(double* b, double* a, int m, int n)
    {
        delete[] b;
        b = new double[m * n];
        for(int i = 0; i < m; i++)
        {
            for(int j = 0; j < n; j++)
            {
                b[j * m + i] = a[i * n + j];
            }
        }
    }
    static void Add(double* c, double* a, double* b, int n)
    {
        for(int i = 0; i < n; i++)
        {
            c[i] = a[i] + b[i];
        }
    }
    static void Scale(double* c, double* a, double b, int n)
    {
        for(int i = 0; i < n; i++)
        {
            c[i] = a[i] * b;
        }
    }
};

#endif
