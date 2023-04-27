#include "Dynamical_Model.h"
#include <cmath>

// Forward Declaration:
// class Dynamical_Model;

// compute M matrix:
void Dynamical_Model::compute_M(const Matrix<3,1> q)
{
    double sigma1 = (_R*_m_CoM*sin(q.getElem(2,0)))/2;
    double sigma2 = ((pow(_R,2))*(4*_m_CoM + 3*_m_CoW))/12;
    double sigma3 = ((pow(_R,2))*(2*_m_CoM + 3*_m_CoW))/12;
    
    _M(0,0) = sigma2;
    _M(0,1) = sigma3;
    _M(0,2) = sigma1;
    _M(1,0) = sigma3;
    _M(1,1) = sigma2;
    _M(1,2) = sigma1;
    _M(2,0) = sigma1;
    _M(2,1) = sigma1;
    _M(2,2) = (_m_CoW*(pow(_L_W2CoM,2))) - (_m_CoW*_L_W2CoM*_L_body)
        + ((_m_CoW*(pow(_L_W2CoM,2)))/3) + (_m_CoM);
}
    // compute C matrix:
void Dynamical_Model::compute_C(const Matrix<3,1> q, const Matrix<3,1> qdot)
{
    double sigma1 = (_R*_m_CoM*(qdot.getElem(2,0))*cos(q.getElem(2,0)))/2;
    _C(0,0) = 0;
    _C(0,1) = 0;
    _C(0,2) = 0;
    _C(1,0) = 0;
    _C(1,1) = 0;
    _C(1,2) = 0;
    _C(2,0) = -sigma1;
    _C(2,1) = -sigma1;
    _C(2,2) = 0;
}

    // compute G matrix:
void Dynamical_Model::compute_G(const Matrix<3,1> q)
{
    _G(0,0) = 0;
    _G(1,0) = 0;
    _G(2,0) = -(_L_W2CoM*_gconst*_m_CoM*sin(q.getElem(2,0)));
}

    // compute Tau:
void Dynamical_Model::compute_Tau(const Matrix<3,1> qdot, const Matrix<3,1> qddot)
{
    _Tau = (_M*qddot) + (_C*qdot) + _G;
}