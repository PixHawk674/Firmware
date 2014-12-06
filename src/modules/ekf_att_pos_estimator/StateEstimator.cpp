//
//  StateEstimator.cpp
//  
//
//  Created by Timothy Bright on 12/2/14.
//
//

#include <StateEstimator.h>
#include <cmath>

    double _pHat = 0;
    double _qHat = 0;
    double _rHat = 0;
    //static double _staticPresHat = 0;
    //static double _diffPresHat = 0;
    double _hHat = 0;
    double _Vahat = 0;
    double _phiHat = 0;
    double _thetaHat = 0;
    double _pnHat = 0;
    double _peHat = 0;
    double _chiHat = 0;
    double _VgHat = 0;
    double _psiHat = 0;
    double _wnHat = 0;
    double _weHat = 0;
    double _alphaHat = 0;
    double _betaHat = 0;
    double _bxHat = 0;
    double _byHat = 0;
    double _bzHat = 0;

void StateEstimator::Estimate(double* xhat, double* uu, double* P)
{
    double yGyroX      = uu[0];
    double yGyroY      = uu[1];
    double yGyroZ      = uu[2];
    double yAccelX     = uu[3];
    double yAccelY     = uu[4];
    double yAccelZ     = uu[5];

    //double yStaticPres = uu[6];
    //double yDiffPres   = uu[7];
    double altitude    = uu[6];
    double airspeed    = uu[7];

    double yGPSN       = uu[8];
    double yGPSE       = uu[9];
    double yGPSH       = uu[10];
    double yGPSVg      = uu[11];
    double yGPSCourse  = uu[12];
    
    double Ts       = P[0];
    double Ts_gps   = P[1];
    double rho      = P[2];
    
    double alpha    = exp(-225*Ts);
    double alphaGPS = exp(-1.5*Ts_gps);
    
    _pHat = LPF(yGyroX, _pHat, alpha);
    _qHat = LPF(yGyroY, _qHat, alpha);
    _rHat = LPF(yGyroZ, _rHat, alpha);
    
    //_staticPresHat = LPF(yStaticPres, _staticPresHat, alpha);
    //_diffPresHat = LPF(yDiffPres, _diffPresHat, alpha);
    //double _hHat = _staticPresHat / rho / 9.81;
    //double _Vahat = sqrt(2.0 / rho * _diffPresHat);
    
    _pnHat = LPF(yGPSN, _pnHat, alphaGPS);
    _peHat = LPF(yGPSE, _peHat, alphaGPS);
    _chiHat = LPF(yGPSCourse, _chiHat, alphaGPS);
    _VgHat = LPF(yGPSVg, _VgHat, alphaGPS);
    _psiHat = _chiHat;
    
    // Calculate EKF #1
    double* xHat1 = new double[2];
    xHat1[0] = _phiHat;
    xHat1[1] = _thetaHat;
    double* u1 = new double[7];
    u1[0] = yGyroX;
    u1[1] = yGyroY;
    u1[2] = yGyroZ;
    u1[3] = _Vahat;
    u1[4] = yAccelX;
    u1[5] = yAccelY;
    u1[6] = yAccelZ;
    RollPitchEstimator(xHat1, xHat1, u1, Ts, P1, Q1, R1);
    _phiHat = xHat1[0];
    _thetaHat = xHat1[1];
    
    // Calculate EKF #2
    double* xHat2 = new double[7];
    xHat2[0] = _pnHat;
    xHat2[1] = _peHat;
    xHat2[2] = _VgHat;
    xHat2[3] = _chiHat;
    xHat2[4] = _wnHat;
    xHat2[5] = _weHat;
    xHat2[6] = _psiHat;
    double* u2 = new double[9];
    u2[0] = _Vahat;
    u2[1] = _qHat;
    u2[2] = _rHat;
    u2[3] = _phiHat;
    u2[4] = _thetaHat;
    u2[5] = yGPSN;
    u2[6] = yGPSE;
    u2[7] = yGPSVg;
    u2[8] = yGPSCourse;
    CoursePositionEstimator(xHat2, xHat2, u2, Ts, P2, Q2, R2);
    _pnHat  = xHat2[0];
    _peHat  = xHat2[1];
    _VgHat  = xHat2[2];
    _chiHat = xHat2[3];
    _wnHat  = xHat2[4];
    _weHat  = xHat2[5];
    _psiHat = xHat2[6];
    
    // copy outputs
    xhat[0]  = _pnHat;
    xhat[1]  = _peHat;
    xhat[2]  = _hHat;
    xhat[3]  = _Vahat;
    xhat[4]  = _alphaHat;
    xhat[5]  = _betaHat;
    xhat[6]  = _phiHat;
    xhat[7]  = _thetaHat;
    xhat[8]  = _chiHat;
    xhat[9]  = _pHat;
    xhat[10] = _qHat;
    xhat[11] = _rHat;
    xhat[12] = _VgHat;
    xhat[13] = _wnHat;
    xhat[14] = _weHat;
    xhat[15] = _psiHat;
    //xhat[16] = _bxHat;
    //xhat[17] = _byHat;
    //xhat[18] = _bzHat;
}

double StateEstimator::LPF(double x, double y0, double alpha)
{
    return (y0*(1.0 - alpha) + x * alpha);
}

void StateEstimator::RollPitchEstimator(double* xhat,
                                        double* x0,
                                        double* u,
                                        double Ts,
                                        double* P,
                                        double* Q,
                                        double* R)
{
    int divs = 10;
    
    // initialize xhat
    for(int i = 0; i < 2; i++)
        xhat[i] = x0[i];
    
    double ts = Ts / divs;  // small timestep
    
    // input values
    double p  = u[0];
    double q  = u[1];
    double r  = u[2];
    double Va = u[3];
    
    double ax = u[4];
    double ay = u[5];
    double az = u[6];
    
    
    double* A  = new double[2*2];
    double* At = new double[2*2];
    double* A1 = new double[2*2];
    double* A2 = new double[2*2];
    double* B  = new double[2*2];
    
    // advance the state using predictive model
    for(int i = 0; i < divs; i++)
    {
        double phi   = xhat[0];
        double theta = xhat[1];
        
        xhat[0] += ts * (p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta));
        xhat[1] += ts * (q*cos(phi)-r*sin(phi));
        
        phi   = xhat[0];
        theta = xhat[1];
        
        for(int j=0;j<4;j++)
            A[j] = 0;           // reset the array for safety
        
        A[0] = q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta);
        A[1] = (q*sin(phi)-r*cos(phi)) / (cos(phi)*cos(phi));
        A[2] = -q*sin(phi)-r*cos(phi);
        
        Matrix::Multiply(A1,A,P,2,2,2);
        Matrix::Transpose(At,A,2,2);
        Matrix::Multiply(A2,P,At,2,2,2);
        Matrix::Add(B,A1,A2,4);
        Matrix::Add(B,B,Q,4);
        Matrix::Scale(B,B,ts,4);
        Matrix::Add(P,P,B,4);
    }
    
    // correct the prediction in the model
    double* C = new double[3*2];
    double* Ci = new double[2];
    double* L = new double[2];
    double* L1 = new double[2];
    double* Pd = new double[2*2];
    
    double* y = new double[3];
    y[0] = ax;
    y[1] = ay;
    y[2] = az;
    
    double* h = new double[3];
    
    for(int i = 0; i < 3; i++)
    {
        phi   = xhat[0];
        theta = xhat[1];
        
        C[0] = 0;
        C[1] = (q*Va+g)*cos(theta);
        C[2] = -g*cos(phi)*cos(theta);
        C[3] = -r*Va*sin(theta)-p*Va*cos(theta)+g*sin(phi)*sin(theta);
        C[4] = g*sin(phi)*cos(theta);
        C[5] = (q*Va + g*cos(phi))*sin(theta);
        
        // select the ith row of dh/dx
        for(int j=0;j<2;j++)
            Ci[j] = C[i*2+j]; // 1x2 matrix
        
        Matrix::Multiply(L,P,Ci,2,2,1);     // P*Ci'
        Matrix::Multiply(L1,Ci,L,1,2,1);    // Ci*L
        double den = L1[0] + R[i*2+i];      // Ci*P*Ci' + R
        Matrix::Scale(L,L,1.0/den,2);       // P*Ci' / den
        
        Matrix::Multiply(Pd,L,Ci,2,1,2);    // L*Ci
        for(int j = 0; j < 2; j++)          // I - L*Ci
        {
            for(int k = 0; k < 2; k++)
            {
                int index = j * 2 + k;
                if(index % 3 == 0)
                    Pd[index] = 1.0 - Pd[index];
                else
                    Pd[index] = -Pd[index];
            }
        }
        Matrix::Multiply(P,Pd,P,2,2,2);     // P = (I-L*Ci)*P
        h[0] = q*Va*sin(theta)+g*sin(theta);
        h[1] = r*Va*cos(theta)-p*Va*sin(theta)-g*cos(theta)*sin(phi);
        h[2] = -q*Va*cos(theta)-g*cos(theta)*cos(phi);
        
        Matrix::Scale(L,L,y[i]-h[i],2);
        Matrix::Add(xhat,xhat,L,2);
    }
}

void StateEstimator::CoursePositionEstimator(double* xhat,
                                             double* x0,
                                             double* u,
                                             double Ts,
                                             double* P,
                                             double* Q,
                                             double* R)
{
    int divs = 10;
    
    // initialize the value of xhat
    for(int i=0;i<7;i++)
        xhat[i] = x0[i];
    
    // Small timestep
    double ts = Ts / divs;
    
    // input values
    double Va    =  u[0];
    double q     =  u[1];
    double r     =  u[2];
    double phi   =  u[3];
    double theta =  u[4];
    
    double gps_n   = u[5];
    double gps_e   = u[6];
    double gps_Vg  = u[7];
    double gps_chi = u[8];

    double* A = new double[7*7];
    double* At = new double[7*7];
    double* A1 = new double[7*7];
    double* A2 = new double[7*7];
    double* B = new double[7*7];
    for (int i = 0; i < divs; i++)
    {
        // advance the state using predictive model
        double pn = xhat[0];
        double pe = xhat[1];
        double Vg = xhat[2];
        double chi = xhat[3];
        double wn = xhat[4];
        double we = xhat[5];
        double psi = xhat[6];
        
        double psidot = q*(sin(phi)/cos(theta)) + r*(cos(phi)/cos(theta));
        
        xhat[0] += ts * Vg*cos(theta);
        xhat[1] += ts * Vg*sin(theta);
        xhat[2] += ts * ((Va*cos(psi)+wn)*(-Va*psidot*sin(psi))+(Va*sin(psi)+we)*(Va*psidot*cos(psi))) / Vg;
        xhat[3] += ts * (g/Vg)*tan(phi)*cos(chi-psi);
        //xhat[4] += ts * 0;
        //xhat[5] += ts * 0;
        xhat[6] += ts * psidot;
        
        // update the covariance matrix
        pn = xhat[0];
        pe = xhat[1];
        Vg = xhat[2];
        chi = xhat[3];
        wn = xhat[4];
        we = xhat[5];
        psi = xhat[6];
        
        for(int j=0;j<49;j++)
            A[j] = 0;           // reset the array for safety
        A[2]  = cos(chi);
        A[3]  = -Vg*sin(chi);
        A[9]  = sin(chi);
        A[10] = Vg*cos(chi);
        A[16] = -((Va*cos(phi)+wn)*(-Va*psidot*sin(psi))+(Va*sin(psi)+we)*(Va*psidot*cos(psi)))/(Vg*Vg);
        A[18] = -psidot*Va*sin(psi);
        A[19] = psidot*Va*cos(psi);
        A[20] = -psidot*Va*(wn*cos(psi)+we*sin(psi))/Vg;
        A[23] = -g/(Vg*Vg)*tan(phi)*cos(chi-psi);
        A[24] = -g/Vg*tan(phi)*sin(chi-psi);
        A[27] = g/Vg*tan(phi)*sin(chi-psi);
        
        Matrix::Multiply(A1,A,P,7,7,7);
        Matrix::Transpose(At,A,7,7);
        Matrix::Multiply(A2,P,At,7,7,7);
        Matrix::Add(B,A1,A2,49);
        Matrix::Add(B,B,Q,49);
        Matrix::Scale(B,B,ts,49);
        Matrix::Add(P,P,B,49);
    }
    
    // correct the prediction in the model
    double* C = new double[6*7];
    double* Ci = new double[7];
    double* L = new double[7];
    double* L1 = new double[1];
    double* Pd = new double[7*7];
    
    double* y = new double[6];
    y[0] = gps_n;
    y[1] = gps_e;
    y[2] = gps_Vg;
    y[3] = gps_chi;
    y[4] = 0;
    y[5] = 0;
    
    double* h = new double[6];
    
    for(int i=0;i<6;i++)
    {
        pn = xhat[0];
        pe = xhat[1];
        Vg = xhat[2];
        chi = xhat[3];
        wn = xhat[4];
        we = xhat[5];
        psi = xhat[6];
        
        // create dh/dx
        for(int j=0;j<42;j++)
        {
            if(j % 8 == 0)
                C[j] = 1;
            else
                C[j] = 0;
        }
        C[30] = -cos(chi);
        C[31] = Vg*sin(chi);
        C[34] = -Va*sin(psi);
        C[37] = -sin(chi);
        C[38] = -Vg*cos(chi);
        C[41] = Va*cos(psi);
        
        // select the ith row of dh/dx
        for(int j=0;j<7;j++)
            Ci[j] = C[i*7+j]; // 1x6 matrix
        
        Matrix::Multiply(L,P,Ci,7,7,1);     // P*Ci'
        Matrix::Multiply(L1,Ci,L,1,7,1);    // Ci*L
        double den = L1[0] + R[i*7+i];      // Ci*P*Ci' + R
        Matrix::Scale(L,L,1.0/den,7);       // P*Ci' / den
        
        Matrix::Multiply(Pd,L,Ci,7,1,7);    // L*Ci
        for(int j = 0; j < 7; j++)          // I - L*Ci
        {
            for(int k = 0; k < 7; k++)
            {
                int index = j * 7 + k;
                if(index % 8 == 0)
                    Pd[index] = 1.0 - Pd[index];
                else
                    Pd[index] = -Pd[index];
            }
        }
        Matrix::Multiply(P,Pd,P,7,7,7);     // P = (I-L*Ci)*P
        h[0] = pn;
        h[1] = pe;
        h[2] = Vg;
        h[3] = chi;
        h[4] = Va*cos(phi)+wn-Vg*cos(chi);
        h[5] = Va*sin(phi)+we-Vg*sin(chi);
        Matrix::Scale(L,L,y[i]-h[i],7);
        Matrix::Add(xhat,xhat,L,7);
    }
}
