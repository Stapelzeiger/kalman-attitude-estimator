#include "template_kalman.h"
#include "math_helpers.h"
#include "mekf_gyro_acc_mag.h"


MEKFGyroAccMag::MEKFGyroAccMag()
{
    this->reset();
    this->Q = this->Q.setIdentity() * 0.000001;
    this->R_mag = this->R_mag.setIdentity() * 1000;
    this->R_grav = this->R_grav.setIdentity() * 1000;
}

MEKFGyroAccMag::~MEKFGyroAccMag()
{

}

void MEKFGyroAccMag::update_imu(const float *gyro, const float *acc, float delta_t)
{
    Eigen::Map<const Eigen::Vector3f> gyro_vec(gyro);
    Eigen::Map<const Eigen::Vector3f> acc_vec(acc);

    // // rotates inertial x to expected measurement (90deg pitch up)
    // const float z_meas_ref[4] = {0.7071067812, 0, 0.7071067812, 0};
    // Eigen::Matrix<float, 2, 1> z;
    // code_gen::measurement_from_vect(acc_v.normalized().data(), this->q_ref.data(), z_meas_ref, z.data());

    // // error quaternion transfer


    // // reference propagation
    // q_ref_cpy = this->q_ref;
    // code_gen::ref_q_propagate(q_ref_cpy.data(), gyro, this->x.data(), delta_t, this->q_ref.data());
    // // todo renormalize?

    // auto f = [delta_t](Eigen::Matrix<float, code_gen::STATE_DIM, 1> &state,
    //             const Eigen::Matrix<float, code_gen::CONTROL_DIM, 1> &control)
    //     {
    //         (void)state;
    //         (void)control;
    //         // Eigen::Matrix<float, code_gen::STATE_DIM, 1> state_cpy = state;
    //         // code_gen::f(state_cpy.data(), control.data(), delta_t, state.data());
    //     };
    // auto F = [delta_t](const Eigen::Matrix<float, code_gen::STATE_DIM, 1> &state,
    //             const Eigen::Matrix<float, code_gen::CONTROL_DIM, 1> &control,
    //             Eigen::Matrix<float, code_gen::STATE_DIM, code_gen::STATE_DIM> &out_jacobian)
    //     {
    //         code_gen::F(state.data(), control.data(), delta_t, out_jacobian.data());
    //     };
    // auto h = [&, z_meas_ref](const Eigen::Matrix<float, code_gen::STATE_DIM, 1> &state,
    //             Eigen::Matrix<float, code_gen::MEASURE_DIM, 1> &pred)
    //     {
    //         code_gen::h(state.data(), this->q_ref.data(), z_meas_ref, pred.data());
    //     };
    // auto H = [&, z_meas_ref](const Eigen::Matrix<float, code_gen::STATE_DIM, 1> &state,
    //             Eigen::Matrix<float, code_gen::MEASURE_DIM, code_gen::STATE_DIM> &out_jacobian)
    //     {
    //         code_gen::H(state.data(), this->q_ref.data(), z_meas_ref, out_jacobian.data());
    //     };

    // ekf_predict<float, code_gen::STATE_DIM, code_gen::CONTROL_DIM>(this->x, this->P, u, this->Q, f, F);
    // ekf_measure<float, code_gen::STATE_DIM, code_gen::MEASURE_DIM>(this->x, this->P, z, this->R, h, H);
}

void MEKFGyroAccMag::update_mag(const float *mag)
{

}

void MEKFGyroAccMag::reset()
{
    this->reset(Eigen::Quaternion<float>::Identity());
}

void MEKFGyroAccMag::reset(Eigen::Quaternionf att)
{
    this->ref_attitude = att;
    this->x.setZero();
    this->P.setIdentity();
    this->P *= 0.001;
}

Eigen::Quaternionf MEKFGyroAccMag::get_attitude()
{
    return this->ref_attitude;
}

void MEKFGyroAccMag::apply_att_err_to_ref(void)
{
    Eigen::Quaternionf delta_q_of_a(2, this->x[0], this->x[1], this->x[2]); // this is unnormalized!
    this->ref_attitude = this->ref_attitude * delta_q_of_a;
    this->ref_attitude.normalize(); // normalize after multiplication
    this->x.block(0, 0, 3, 1).setZero();
}

Eigen::Quaternionf MEKFGyroAccMag::vect_measurement_basis(Eigen::Vector3f expected)
{
    Eigen::Quaternionf transformation;
    transformation.setFromTwoVectors(expected, Eigen::Vector3f(1, 0, 0));
    return transformation;
}

Eigen::Matrix3f MEKFGyroAccMag::vect_measurement_basis_b_frame(Eigen::Vector3f expected)
{
    Eigen::Quaternionf I_to_m = MEKFGyroAccMag::vect_measurement_basis(expected);
    return (I_to_m * this->ref_attitude).toRotationMatrix();
}

Eigen::Matrix<float, 2, 1> MEKFGyroAccMag::vect_measurement_transform(Eigen::Matrix3f measurement_basis, Eigen::Vector3f measurement_b)
{
    Eigen::Matrix<float, 2, 3>v_m_to_z;
    v_m_to_z << 0, 1, 0,
                0, 0, 1;
    return v_m_to_z * measurement_basis * measurement_b;
}

Eigen::Matrix<float, 2, 3> MEKFGyroAccMag::vect_measurement_jacobian(Eigen::Matrix3f measurement_basis, Eigen::Vector3f expected_b)
{
    Eigen::Matrix<float, 2, 3>v_m_to_z;
    v_m_to_z << 0, 1, 0,
                0, 0, 1;
    return v_m_to_z * measurement_basis * cross_product_matrix(expected_b.normalized());
}

void MEKFGyroAccMag::measure_vect(Eigen::Vector3f expected, Eigen::Vector3f measured)
{
    Eigen::Matrix3f b_to_m = this->vect_measurement_basis_b_frame(expected);
    Eigen::Vector3f expected_b = this->ref_attitude.conjugate()._transformVector(expected);
    Eigen::Vector2f z = vect_measurement_transform(b_to_m, measured);
    Eigen::Matrix<float, 2, 3> Ha = vect_measurement_jacobian(b_to_m, expected_b);


    
}

