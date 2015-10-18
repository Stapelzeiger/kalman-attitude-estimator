#ifndef TEMPLATE_KALMAN_H
#define TEMPLATE_KALMAN_H

#include <Eigen/Dense>

template<typename scalar, int state_dim, int control_dim>
void ekf_predict(Eigen::Matrix<scalar, state_dim, 1> &state,
                 Eigen::Matrix<scalar, state_dim, state_dim> &state_cov,
                 const Eigen::Matrix<scalar, control_dim, 1> &control,
                 void (*state_prop_fn)(Eigen::Matrix<scalar, state_dim, 1> &state,
                                       const Eigen::Matrix<scalar, control_dim, 1> &control),
                 void (*state_prop_jacobian_fn)(const Eigen::Matrix<scalar, state_dim, 1> &state,
                                                const Eigen::Matrix<scalar, control_dim, 1> &control,
                                                Eigen::Matrix<scalar, state_dim, state_dim> &out_jacobian)
                 )
{
    Eigen::Matrix<scalar, state_dim, state_dim> F;
    state_prop_jacobian_fn(state, control, F);
    state_prop_fn(state, control);
}


#endif /* TEMPLATE_KALMAN_H */