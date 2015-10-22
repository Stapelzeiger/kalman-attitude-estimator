#include <iostream>
#include "state_estimator.h"
#include "template_kalman.h"

namespace simple_ekf {
#include "../simple_ekf.h"
}

StateEstimator::StateEstimator()
{
    this->P.setIdentity();
    this->P *= 0.0001;
    this->reset();
}

StateEstimator::~StateEstimator()
{

}

#define DELTA_T 0.001f

void StateEstimator::update_imu(const float *gyro, const float *acc, float delta_t)
{
    // Eigen::Matrix3f Q(Eigen::Vector3f(0.1,0.1,0.1,0.1, 0.01, 0.01, 0.01).asDiagonal());
    Eigen::Matrix<float, simple_ekf::STATE_DIM, simple_ekf::STATE_DIM> Q;
    Q = Q.setIdentity() * 0.0001;
    Eigen::Matrix<float, simple_ekf::MEASURE_DIM, simple_ekf::MEASURE_DIM> R;
    R = R.setIdentity() * 10000;
    Eigen::Map<const Eigen::Vector3f> u(gyro);
    Eigen::Map<const Eigen::Vector3f> z(acc);

    auto f = [](Eigen::Matrix<float, simple_ekf::STATE_DIM, 1> &state,
                const Eigen::Matrix<float, simple_ekf::CONTROL_DIM, 1> &control)
        {
            Eigen::Matrix<float, simple_ekf::STATE_DIM, 1> state_cpy = state;
            simple_ekf::f(DELTA_T, state_cpy.data(), control.data(), state.data());
        };
    auto F = [](const Eigen::Matrix<float, simple_ekf::STATE_DIM, 1> &state,
                const Eigen::Matrix<float, simple_ekf::CONTROL_DIM, 1> &control,
                Eigen::Matrix<float, simple_ekf::STATE_DIM, simple_ekf::STATE_DIM> &out_jacobian)
        {
            simple_ekf::F(DELTA_T, state.data(), control.data(), out_jacobian.data());
        };
    auto h = [](const Eigen::Matrix<float, simple_ekf::STATE_DIM, 1> &state,
                Eigen::Matrix<float, simple_ekf::MEASURE_DIM, 1> &pred)
        {
            simple_ekf::h(state.data(), pred.data());
        };
    auto H = [](const Eigen::Matrix<float, simple_ekf::STATE_DIM, 1> &state,
                Eigen::Matrix<float, simple_ekf::MEASURE_DIM, simple_ekf::STATE_DIM> &out_jacobian)
        {
            simple_ekf::H(state.data(), out_jacobian.data());
        };


    // float z_est[3];
    // simple_ekf::h(this->x.data(), &z_est[0]);
    // std::cerr << z_est[0] << ", "
    //           << z_est[1] << ", "
    //           << z_est[2] << std::endl;

    ekf_predict<float, simple_ekf::STATE_DIM, simple_ekf::CONTROL_DIM>(this->x, this->P, u, Q, f, F);
    this->x.topLeftCorner(4, 1) = this->x.topLeftCorner(4, 1).normalized();
    ekf_measure<float, simple_ekf::STATE_DIM, simple_ekf::MEASURE_DIM>(this->x, this->P, z, R, h, H);
    this->x.topLeftCorner(4, 1) = this->x.topLeftCorner(4, 1).normalized();
}

void StateEstimator::reset()
{
    this->reset(Eigen::Quaternion<float>::Identity());
}

void StateEstimator::reset(Eigen::Quaternionf att)
{
    this->x.setZero();
    this->x(0, 0) = att.w();
    this->x(1, 0) = att.x();
    this->x(2, 0) = att.y();
    this->x(3, 0) = att.z();
}

Eigen::Quaternionf StateEstimator::get_attitude()
{
    return Eigen::Quaternionf(this->x(0, 0), this->x(1, 0), this->x(2, 0), this->x(3, 0));
}
