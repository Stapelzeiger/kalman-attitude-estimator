#define EIGEN_RUNTIME_NO_MALLOC
#include <iostream>
#include "template_kalman.h"
#include "CppUTestExt/MockSupport.h"
#include "CppUTest/TestHarness.h" // this include must be last

using namespace std;
using namespace Eigen;

template <typename scalar, int rows, int cols>
class EigenMatrixComparator : public MockNamedValueComparator
{
public:
    virtual bool isEqual(const void* matrix1, const void* matrix2)
    {
        return ((const Eigen::Matrix<scalar, rows, cols>*)matrix1)->isApprox(*(const Eigen::Matrix<scalar, rows, cols>*)matrix2);
    }
    virtual SimpleString valueToString(const void* matrix)
    {
        ostringstream s;
        s << endl << *(const Eigen::Matrix<scalar, rows, cols>*)matrix << endl;
        return StringFrom(s.str());
    }
};


TEST_GROUP(TemplateEKFPredict)
{
    void setup(void)
    {
        // The template based Kalman should never dynamically allocate memory
        Eigen::internal::set_is_malloc_allowed(false);
    }

    void teardown(void)
    {
        mock().clear();
        mock().removeAllComparatorsAndCopiers();
    }
};


TEST(TemplateEKFPredict, StatePropagationAndJacobianCalled)
{
    EigenMatrixComparator<float, 3, 1> V3fcomparator;
    mock().installComparator("Vector3f", V3fcomparator);
    EigenMatrixComparator<float, 1, 1> V1fcomparator;
    mock().installComparator("Vector1f", V1fcomparator);

    Vector3f x;
    x << 1, 2, 3;
    Matrix<float, 1, 1> u;
    u << 4;
    Matrix3f P;
    P.setIdentity();
    Matrix3f Q;
    Q.setIdentity();
    auto f = [](Eigen::Matrix<float, 3, 1> &state,
                const Eigen::Matrix<float, 1, 1> &control)
        {
            mock().actualCall("f")
                .withParameterOfType("Vector3f", "state", (void*)&state)
                .withParameterOfType("Vector1f", "control", (void*)&control);
        };
    auto F = [](const Eigen::Matrix<float, 3, 1> &state,
                const Eigen::Matrix<float, 1, 1> &control,
                Eigen::Matrix<float, 3, 3> &out_jacobian)
        {
            mock().actualCall("F")
                .withParameterOfType("Vector3f", "state", (void*)&state)
                .withParameterOfType("Vector1f", "control", (void*)&control);
        };

    mock().expectOneCall("f")
        .withParameterOfType("Vector3f", "state", (void*)&x)
        .withParameterOfType("Vector1f", "control", (void*)&u);
    mock().expectOneCall("F")
        .withParameterOfType("Vector3f", "state", (void*)&x)
        .withParameterOfType("Vector1f", "control", (void*)&u);

    ekf_predict<float, 3, 1>(x, P, u, Q, f, F);

    mock().checkExpectations();
}


TEST(TemplateEKFPredict, PredictWorks)
{
    Vector3f x;
    x << 1, 2, 3;
    Matrix3f P0(3, 3);
    P0.setIdentity();
    Matrix3f P = P0;
    Matrix<float, 1, 1> u(1);
    u << 0;
    Matrix3f F0;
    F0 << 0.9, 1, 0,
          0, 0.9, 1,
          0, 0, 0.9;
    Matrix3f Q;
    Q = Q.setIdentity() * 0.1;
    Vector3f x1;
    x1 << 4, 5, 6;
    auto f = [](Eigen::Matrix<float, 3, 1> &state,
                const Eigen::Matrix<float, 1, 1> &control)
        {
            mock().actualCall("f").withOutputParameter("new_state", &state);
        };
    auto F = [](const Eigen::Matrix<float, 3, 1> &state,
                const Eigen::Matrix<float, 1, 1> &control,
                Eigen::Matrix<float, 3, 3> &out_jacobian)
        {
            mock().actualCall("F").withOutputParameter("jacobian", &out_jacobian);
        };

    mock().expectOneCall("f")
        .withOutputParameterReturning("new_state", &x1, sizeof(x1));
    mock().expectOneCall("F")
        .withOutputParameterReturning("jacobian", &F0, sizeof(F0));

    ekf_predict<float, 3, 1>(x, P, u, Q, f, F);

    CHECK_TRUE(x.isApprox(x1));
    CHECK_TRUE(P.isApprox(F0 * P0 * F0.transpose() + Q));

    mock().checkExpectations();
}

