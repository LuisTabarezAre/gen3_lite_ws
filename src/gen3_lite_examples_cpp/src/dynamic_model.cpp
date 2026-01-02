#include "components_cpp/dynamic_model.hpp"
#include <cmath>
////////// Physical Parameters /////////

/// Mass [kg]  URDF ///
constexpr double m2 = 1.17756164;
constexpr double m3 = 0.59767669;
constexpr double m4 = 0.52693412;
constexpr double m5 = 0.58097325;
constexpr double m6 = 0.20175692;

/// Lengths [m] URDF ///
constexpr double a2 = 0.280; 
constexpr double d4 = 0.245;
constexpr double d5 = 0.570;
constexpr double d6 = 0.235; 

/// Center of mass [m] URDF///
constexpr double rcx2 = -0.02998299;
constexpr double rcy2 =  0.21154808;
constexpr double rcx3 = -0.03015590;
constexpr double rcz3 = -0.00735550;
constexpr double rcx4 =  0.00575149;
constexpr double rcy4 = -0.01000443;
constexpr double rcz4 =  0.08719207;
constexpr double rcx5 = -0.08056517;
constexpr double rcy5 = -0.00980409;
constexpr double rcz5 =  0.01872799;
constexpr double rcx6 = -0.00993041;
constexpr double rcy6 = -0.00994950;
constexpr double rcz6 =  0.06135883;

/// Gravity m/s² /// 
constexpr double g = -9.8000000000000007;

///// Gravity vector deltas/////
constexpr double dlt1 = g*(a2*(m2 + m3 + m4 + m5 + m6) + m2*rcx2);
constexpr double dlt2 = g*(d4*(m4 + m5 + m6) + m4*rcy4 + m3*rcz3);
constexpr double dlt3 = g*(d6*m6 + m5*rcz5 + m6*rcz6);
constexpr double dlt4 = g*m5*rcx5;
constexpr double dlt5 = g*m6*rcy6;
constexpr double dlt6 = g*m6*rcx6;
constexpr double dlt7 = g*m3*rcx3;
constexpr double dlt8 = g*(d5*(m5+m6) + m5*rcy5 + m4*rcz4);
constexpr double dlt9 = g*m4*rcx4;
constexpr double dlt10 = g*m2*rcy2;

std::vector<double> calc_gravity(const std::vector<double>& q)
{
    std::vector<double> G(6,0.0);

    const double& q1 = q[1];
    const double& q2 = q[2];
    const double& q3 = q[3];
    const double& q4 = q[4];
    const double& q5 = q[5];

    G[0] = 0.0;
    G[1] = dlt10*std::cos(q1) - dlt1*std::sin(q1) + dlt7*std::cos(q1)*std::cos(q2) + dlt2*std::cos(q1)*std::sin(q2) - dlt2*std::cos(q2)*std::sin(q1) + dlt7*std::sin(q1)*std::sin(q2) + dlt8*std::cos(q1)*std::cos(q2)*std::cos(q3) + dlt3*std::cos(q1)*std::cos(q4)*std::sin(q2) - dlt3*std::cos(q2)*std::cos(q4)*std::sin(q1) - dlt9*std::cos(q1)*std::cos(q2)*std::sin(q3) - dlt4*std::cos(q1)*std::sin(q2)*std::sin(q4) + dlt4*std::cos(q2)*std::sin(q1)*std::sin(q4) + dlt8*std::cos(q3)*std::sin(q1)*std::sin(q2) - dlt9*std::sin(q1)*std::sin(q2)*std::sin(q3) + dlt6*std::cos(q1)*std::cos(q2)*std::cos(q3)*std::cos(q5) + dlt4*std::cos(q1)*std::cos(q2)*std::cos(q4)*std::sin(q3) - dlt5*std::cos(q1)*std::cos(q2)*std::cos(q3)*std::sin(q5) + dlt3*std::cos(q1)*std::cos(q2)*std::sin(q3)*std::sin(q4) + dlt5*std::cos(q1)*std::cos(q5)*std::sin(q2)*std::sin(q4) - dlt5*std::cos(q2)*std::cos(q5)*std::sin(q1)*std::sin(q4) + dlt6*std::cos(q3)*std::cos(q5)*std::sin(q1)*std::sin(q2) + dlt4*std::cos(q4)*std::sin(q1)*std::sin(q2)*std::sin(q3) - dlt5*std::cos(q3)*std::sin(q1)*std::sin(q2)*std::sin(q5) + dlt6*std::cos(q1)*std::sin(q2)*std::sin(q4)*std::sin(q5) - dlt6*std::cos(q2)*std::sin(q1)*std::sin(q4)*std::sin(q5) + dlt3*std::sin(q1)*std::sin(q2)*std::sin(q3)*std::sin(q4) - dlt6*std::cos(q1)*std::cos(q2)*std::cos(q4)*std::sin(q3)*std::sin(q5) - dlt5*std::cos(q4)*std::cos(q5)*std::sin(q1)*std::sin(q2)*std::sin(q3) - dlt6*std::cos(q4)*std::sin(q1)*std::sin(q2)*std::sin(q3)*std::sin(q5) - dlt5*std::cos(q1)*std::cos(q2)*std::cos(q4)*std::cos(q5)*std::sin(q3);
    G[2] = dlt2*std::cos(q2)*std::sin(q1) - dlt2*std::cos(q1)*std::sin(q2) - dlt7*std::cos(q1)*std::cos(q2) - dlt7*std::sin(q1)*std::sin(q2) - dlt8*std::cos(q1)*std::cos(q2)*std::cos(q3) - dlt3*std::cos(q1)*std::cos(q4)*std::sin(q2) + dlt3*std::cos(q2)*std::cos(q4)*std::sin(q1) + dlt9*std::cos(q1)*std::cos(q2)*std::sin(q3) + dlt4*std::cos(q1)*std::sin(q2)*std::sin(q4) - dlt4*std::cos(q2)*std::sin(q1)*std::sin(q4) - dlt8*std::cos(q3)*std::sin(q1)*std::sin(q2) + dlt9*std::sin(q1)*std::sin(q2)*std::sin(q3) - dlt6*std::cos(q1)*std::cos(q2)*std::cos(q3)*std::cos(q5) - dlt4*std::cos(q1)*std::cos(q2)*std::cos(q4)*std::sin(q3) + dlt5*std::cos(q1)*std::cos(q2)*std::cos(q3)*std::sin(q5) - dlt3*std::cos(q1)*std::cos(q2)*std::sin(q3)*std::sin(q4) - dlt5*std::cos(q1)*std::cos(q5)*std::sin(q2)*std::sin(q4) + dlt5*std::cos(q2)*std::cos(q5)*std::sin(q1)*std::sin(q4) - dlt6*std::cos(q3)*std::cos(q5)*std::sin(q1)*std::sin(q2) - dlt4*std::cos(q4)*std::sin(q1)*std::sin(q2)*std::sin(q3) + dlt5*std::cos(q3)*std::sin(q1)*std::sin(q2)*std::sin(q5) - dlt6*std::cos(q1)*std::sin(q2)*std::sin(q4)*std::sin(q5) + dlt6*std::cos(q2)*std::sin(q1)*std::sin(q4)*std::sin(q5) - dlt3*std::sin(q1)*std::sin(q2)*std::sin(q3)*std::sin(q4) + dlt6*std::cos(q1)*std::cos(q2)*std::cos(q4)*std::sin(q3)*std::sin(q5) + dlt5*std::cos(q4)*std::cos(q5)*std::sin(q1)*std::sin(q2)*std::sin(q3) + dlt6*std::cos(q4)*std::sin(q1)*std::sin(q2)*std::sin(q3)*std::sin(q5) + dlt5*std::cos(q1)*std::cos(q2)*std::cos(q4)*std::cos(q5)*std::sin(q3);
    G[3] = -std::sin(q1 - q2)*(dlt9*std::cos(q3) + dlt8*std::sin(q3) - dlt4*std::cos(q3)*std::cos(q4) - dlt3*std::cos(q3)*std::sin(q4) + dlt6*std::cos(q5)*std::sin(q3) - dlt5*std::sin(q3)*std::sin(q5) + dlt5*std::cos(q3)*std::cos(q4)*std::cos(q5) + dlt6*std::cos(q3)*std::cos(q4)*std::sin(q5));
    G[4] = dlt5*std::cos(q5)*(std::cos(q1)*std::cos(q2)*std::cos(q4) + std::cos(q4)*std::sin(q1)*std::sin(q2) - std::cos(q1)*std::sin(q2)*std::sin(q3)*std::sin(q4) + std::cos(q2)*std::sin(q1)*std::sin(q3)*std::sin(q4)) - dlt4*(std::cos(q1)*std::cos(q2)*std::cos(q4) + std::cos(q4)*std::sin(q1)*std::sin(q2) - std::cos(q1)*std::sin(q2)*std::sin(q3)*std::sin(q4) + std::cos(q2)*std::sin(q1)*std::sin(q3)*std::sin(q4)) - dlt3*(std::sin(q1)*std::sin(q2)*std::sin(q4) + std::cos(q1)*std::cos(q2)*std::sin(q4) + std::cos(q1)*std::cos(q4)*std::sin(q2)*std::sin(q3) - std::cos(q2)*std::cos(q4)*std::sin(q1)*std::sin(q3)) + dlt6*std::sin(q5)*(std::cos(q1)*std::cos(q2)*std::cos(q4) + std::cos(q4)*std::sin(q1)*std::sin(q2) - std::cos(q1)*std::sin(q2)*std::sin(q3)*std::sin(q4) + std::cos(q2)*std::sin(q1)*std::sin(q3)*std::sin(q4));
    G[5] = dlt6*(std::cos(q1)*std::cos(q2)*std::cos(q5)*std::sin(q4) + std::cos(q1)*std::cos(q3)*std::sin(q2)*std::sin(q5) - std::cos(q2)*std::cos(q3)*std::sin(q1)*std::sin(q5) + std::cos(q5)*std::sin(q1)*std::sin(q2)*std::sin(q4) + std::cos(q1)*std::cos(q4)*std::cos(q5)*std::sin(q2)*std::sin(q3) - std::cos(q2)*std::cos(q4)*std::cos(q5)*std::sin(q1)*std::sin(q3)) - dlt5*(std::cos(q2)*std::cos(q3)*std::cos(q5)*std::sin(q1) - std::cos(q1)*std::cos(q3)*std::cos(q5)*std::sin(q2) + std::cos(q1)*std::cos(q2)*std::sin(q4)*std::sin(q5) + std::sin(q1)*std::sin(q2)*std::sin(q4)*std::sin(q5) + std::cos(q1)*std::cos(q4)*std::sin(q2)*std::sin(q3)*std::sin(q5) - std::cos(q2)*std::cos(q4)*std::sin(q1)*std::sin(q3)*std::sin(q5));
    
    G[1] = (G[1]*(-1.0));

    return G;
}