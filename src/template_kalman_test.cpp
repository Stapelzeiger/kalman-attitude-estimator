#define EIGEN_RUNTIME_NO_MALLOC
#include <iostream>
#include "template_kalman.h"
#include "CppUTestExt/MockSupport.h"
#include "CppUTest/TestHarness.h" // this include must be last

using namespace std;
using namespace Eigen;

TEST_GROUP(TemplateEKF)
{

};

TEST(TemplateEKF, Simple)
{
    Vector3f x(3);
    x << 1, 2, 3;
    Matrix3f P(3, 3);
    P << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    Matrix<float, 1, 1> u(1);
    u << 4;
    auto f = [](Eigen::Matrix<float, 3, 1> &x,
                const Eigen::Matrix<float, 1, 1> &u)
        {
            // Matrix<float, 3, 1> c;
            // c << 10, 0, 0;
            // x = x + c * u;
            return;
        };
    auto F = [](const Eigen::Matrix<float, 3, 1> &state,
                const Eigen::Matrix<float, 1, 1> &control,
                Eigen::Matrix<float, 3, 3> &out_jacobian)
        {
            return;
        };
    Eigen::internal::set_is_malloc_allowed(false);
    ekf_predict<float, 3, 1>(x, P, u, f, F);
    Eigen::internal::set_is_malloc_allowed(true);
}


