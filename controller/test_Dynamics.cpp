#include <iostream>
#include "Dynamical_Model.h"
#include "Matrix.h"

int main()
{
    // Initialization:
    Dynamical_Model dm;
    Matrix<3,1> q, dq, ddq, torque, g_matrix;
    Matrix<3,3> m_matrix, c_matrix;

    // Test with 
    // q = [0.0; 0.0; 0.1]
    // dq = [0.05; 0.05; 0.05]
    // ddq = [0.01; 0.01; 0.01] :
    q(2,0) = 0.1;
    dq += 0.05;
    ddq += 0.01;

    // Use Dynamical_Model object to compute torque:
    dm.compute_M(q);
    m_matrix = dm.get_M();
    dm.compute_C(q, dq);
    c_matrix = dm.get_C();
    dm.compute_G(q);
    g_matrix = dm.get_G();
    dm.compute_Tau(dq, ddq);
    torque = dm.get_Tau();

    // Print to cout:
    std::cout << "M_matrix = \n" << m_matrix.toString() << std::endl;
    std::cout << "C_matrix = \n" << c_matrix.toString() << std::endl;
    std::cout << "G_matrix = \n" << g_matrix.toString() << std::endl;
    std::cout << "torque = \n" << torque.toString() << std::endl;

    return 0;
}