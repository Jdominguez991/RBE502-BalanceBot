#ifndef DYNAMICAL_MODEL_H
#define DYNAMICAL_MODEL_H

#include "Matrix.h"

class Dynamical_Model
{
private:
    Matrix<3, 3> _M;                // M matrix
    Matrix<3, 3> _C;                // C matrix
    Matrix<3, 1> _G;                // G matrix
    Matrix<3, 1> _Tau;              // System Torque
    const double _R = 0.2;          // wheel radius
    const double _m_CoM = 30.0;     // mass of upper block
    const double _m_CoW = 2.0;      // mass of both wheels
    const double _L_body = 0.8;     // length of entire teeterbot block without wheels
    const double _L_W2CoM = 0.65;   // length from wheels to teeterbot center of mass - MAY NEED TO CALCULATE AND UPDATE THIS LATER
    const double _L_wheels = 0.3;   // width of the teeterbot (length from wheel to wheel)
    const double _gconst = 9.81;    // gravity constant

public:
    // Constructor:
    Dynamical_Model()
    {   
        // This can also be used to "clear" an existing dynamical Model
        _M *= 0;
        _C *= 0;
        _G *= 0;
        _Tau *= 0;
    }

    // compute M matrix:
    void compute_M(const Matrix<3,1> q);

    // compute C matrix:
    void compute_C(const Matrix<3,1> q, const Matrix<3,1> qdot);

    // compute G matrix:
    void compute_G(const Matrix<3,1> q);

    // compute Tau:
    void compute_Tau(const Matrix<3,1> qdot, const Matrix<3,1> qddot);

    // Getters:
    Matrix<3,3> get_M() { return _M; }
    Matrix<3,3> get_C() { return _C; }
    Matrix<3,1> get_G() { return _G; }
    Matrix<3,1> get_Tau() { return _Tau; }
};

#endif