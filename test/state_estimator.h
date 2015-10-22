#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace _simple_ekf {
    #include "../simple_ekf.h"
}

class StateEstimator
{
public:
    Eigen::Matrix<float, _simple_ekf::STATE_DIM, 1> x;
    Eigen::Matrix<float, _simple_ekf::STATE_DIM, _simple_ekf::STATE_DIM> P;

    StateEstimator();
    ~StateEstimator();

    void reset();
    void reset(Eigen::Quaternionf att);
    void update_imu(const float *gyro, const float *acc, float delta_t);
    Eigen::Quaternionf get_attitude();

};
