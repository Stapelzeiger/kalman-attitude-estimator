#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace _simple_ekf {
    #include "code_gen/ekf_gyro_acc.h"
}

class StateEstimator
{
public:
    Eigen::Matrix<float, _simple_ekf::STATE_DIM, 1> x;
    Eigen::Matrix<float, _simple_ekf::STATE_DIM, _simple_ekf::STATE_DIM> P;
    Eigen::Matrix<float, _simple_ekf::STATE_DIM, _simple_ekf::STATE_DIM> Q;
    Eigen::Matrix<float, _simple_ekf::MEASURE_DIM, _simple_ekf::MEASURE_DIM> R;

    StateEstimator();
    ~StateEstimator();

    void reset();
    void reset(Eigen::Quaternionf att);
    void update_imu(const float *gyro, const float *acc, float delta_t);
    Eigen::Quaternionf get_attitude();

};
