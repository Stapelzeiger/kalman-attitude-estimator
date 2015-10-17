#include "ExtendedKalmanFilter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(unsigned int dimension,
                                           state_prop_fn f,
                                           state_prop_jacobian_fn F,
                                           measurement_pred_fn h,
                                           measurement_pred_jacobian_fn H)
{

}

ExtendedKalmanFilter::~ExtendedKalmanFilter()
{

}

void ExtendedKalmanFilter::state_propagation(const Eigen::VectorXf &control_vect,
                                             const Eigen::MatrixXf &control_cov)
{
    
}