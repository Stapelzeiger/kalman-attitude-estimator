#include <iostream>
#include "ExtendedKalmanFilter.h"
#include "CppUTestExt/MockSupport.h"
#include "CppUTest/TestHarness.h" // this include must be last

using namespace std;

// system dynamics & measurement mock callbacks

void kalman_state_prop(const Eigen::VectorXf &state,
                       const Eigen::VectorXf &control,
                       Eigen::VectorXf &out_state)
{
    mock().actualCall("kalman_state_prop")
        .withParameterOfType("VectorXf", "state", (void*)&state)
        .withParameterOfType("VectorXf", "control", (void*)&control);
    cout << "state prop called" << endl;
}

void kalman_state_prop_jacobian(const Eigen::VectorXf &state,
                                const Eigen::VectorXf &control,
                                Eigen::MatrixXf &out_jacobian)
{

}

void kalman_measurement_pred(const Eigen::VectorXf &state,
                             Eigen::VectorXf &out_pred)
{

}

void kalman_measurement_pred_jacobian(const Eigen::VectorXf &state,
                                      Eigen::MatrixXf &out_jacobian)
{

}



class VectorXfComparator : public MockNamedValueComparator
{
public:
    virtual bool isEqual(void* object1, void* object2)
    {
        return object1 == object2;
    }
    virtual SimpleString valueToString(void* object)
    {
        return StringFrom(object);
    }
};


TEST_GROUP(ExtendedKalmanFilterTest)
{
    ExtendedKalmanFilter *ekf;

    void setup(void)
    {
        ekf = new ExtendedKalmanFilter(3,
                                       kalman_state_prop,
                                       kalman_state_prop_jacobian,
                                       kalman_measurement_pred,
                                       kalman_measurement_pred_jacobian);

    }

    void teardown(void)
    {
        delete ekf;
    }
};

TEST(ExtendedKalmanFilterTest, StatePropagationCalled)
{
    Eigen::VectorXf u(2);
    u << 1,2;
    Eigen::MatrixXf Q(2,2);
    Q << 1, 0,
         0, 1;

    ekf->state_propagation(u, Q);
}

