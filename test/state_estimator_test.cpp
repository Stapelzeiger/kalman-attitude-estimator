#include "state_estimator.h"
#include "CppUTest/TestHarness.h" // this include must be last


TEST_GROUP(StateEstimator)
{
    StateEstimator s;
    void setup(void)
    {
        s = StateEstimator();
    }
};

TEST(StateEstimator, UpdateIMU)
{
    float gyro[3] = {0, 0, 0};
    float acc[3] = {0, 0 ,0};
    s.update_imu(gyro, acc, 0.1);
}

TEST(StateEstimator, GetAttitude)
{
    CHECK(s.get_attitude().isApprox(Eigen::Quaternionf(1, 0, 0, 0)));
}

