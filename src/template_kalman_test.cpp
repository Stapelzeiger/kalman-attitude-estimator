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
        return *(const Eigen::Matrix<scalar, rows, cols>*)matrix1 == *(const Eigen::Matrix<scalar, rows, cols>*)matrix2;
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


TEST(TemplateEKFPredict, Simple)
{
    Vector3f x(3);
    Matrix3f P(3, 3);
    Matrix<float, 1, 1> u(1);

    auto f = [](Eigen::Matrix<float, 3, 1> &x,
                const Eigen::Matrix<float, 1, 1> &u) {};
    auto F = [](const Eigen::Matrix<float, 3, 1> &state,
                const Eigen::Matrix<float, 1, 1> &control,
                Eigen::Matrix<float, 3, 3> &out_jacobian) {};
    ekf_predict<float, 3, 1>(x, P, u, f, F);
}


TEST(TemplateEKFPredict, StatePropagationCalled)
{
    EigenMatrixComparator<float, 3, 1> V3fcomparator;
    mock().installComparator("Vector3f", V3fcomparator);
    EigenMatrixComparator<float, 1, 1> V1fcomparator;
    mock().installComparator("Vector1f", V1fcomparator);

    Vector3f x(3);
    x << 1, 2, 3;
    Matrix3f P(3, 3);
    P << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    Matrix<float, 1, 1> u(1);
    u << 4;
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

    ekf_predict<float, 3, 1>(x, P, u, f, F);

    mock().checkExpectations();
}


