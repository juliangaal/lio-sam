#include "gravity_factor/gravity_estimator.h"

namespace lio_sam
{
template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> GravityEstimator<T>::TangentBasis( Eigen::Matrix<T, 3, 1> &g0 )
{
  Eigen::Matrix<T, 3, 1> b, c;
  Eigen::Matrix<T, 3, 1> a = g0.normalized();
  Eigen::Matrix<T, 3, 1> tmp( 0, 0, 1 );
  if ( a == tmp )
    tmp << 1, 0, 0;
  b = ( tmp - a * ( a.transpose() * tmp ) ).normalized();
  c = a.cross( b );
  Eigen::MatrixXd bc( 3, 2 );
  bc.block<3, 1>( 0, 0 ) = b;
  bc.block<3, 1>( 0, 1 ) = c;
  return bc;
}

template <typename T>
bool GravityEstimator<T>::ApproximateGravity(
    const std::deque<TransformAndPreintegrator<T>> &all_laser_transforms,
    const Eigen::Transform<T, 3, Eigen::Affine> &   transform_lb,
    const std::deque<Eigen::Matrix<T, 3, 1>> &      Vs,
    const T                                         g_norm,
    Eigen::Matrix<T, 3, 1> &                        g )
{
  size_t                              window_size = all_laser_transforms.size();
  int                                 n_state     = window_size * 3 + 3;  //3g + n*3v
  Eigen::Matrix<T, Eigen::Dynamic, 1> x;
  // Laser in Base(IMU)
  Eigen::Matrix<T, 3, 1> tlb = transform_lb.translation();
  Eigen::Matrix<T, 3, 3> rlb = transform_lb.linear();

  Eigen::Matrix<T, 3, 3> A;
  A.setZero();
  Eigen::Matrix<T, 3, 1> b;
  b.setZero();

  if ( window_size < 3 )
  {
    LOG( WARNING ) << ">>>>>>>Gravity estimate window size is not enough <<<<<<<";
    return false;
  }
  for ( size_t i = 0; i < window_size - 1; ++i )
  {
    const TransformAndPreintegrator<T> *frame_i = &all_laser_transforms[ i ];
    const TransformAndPreintegrator<T> *frame_j = &all_laser_transforms[ i + 1 ];

    if ( !frame_j->pre_integration )
    {
      continue;
    }

    Eigen::Matrix<T, 6, 3> tmp_A;
    tmp_A.setZero();
    Eigen::Matrix<T, 6, 1> tmp_b;
    tmp_b.setZero();

    T                      dt        = frame_j->pre_integration->deltaTij();
    Eigen::Matrix<T, 3, 3> frame_i_R = frame_i->transform.linear();
    Eigen::Matrix<T, 3, 3> frame_j_R = frame_j->transform.linear();
    Eigen::Matrix<T, 3, 1> frame_i_T = frame_i->transform.translation();
    Eigen::Matrix<T, 3, 1> frame_j_T = frame_j->transform.translation();

    tmp_A.block<3, 3>( 0, 0 ) = frame_i_R.transpose() * dt * dt / 2 * Eigen::Matrix<T, 3, 3>::Identity();

    tmp_b.block<3, 1>( 0, 0 ) = frame_j->pre_integration->deltaPij() + frame_i_R.transpose() * frame_j_R * tlb - tlb - frame_i_R.transpose() * ( frame_j_T - frame_i_T ) + dt * Eigen::Matrix<T, 3, 3>::Identity() * Vs[ i ];

    tmp_A.block<3, 3>( 3, 0 ) = frame_i_R.transpose() * dt * Eigen::Matrix<T, 3, 3>::Identity();
    tmp_b.block<3, 1>( 3, 0 ) = frame_j->pre_integration->deltaVij() + Eigen::Matrix<T, 3, 3>::Identity() * Vs[ i ] - frame_i_R.transpose() * frame_j_R * Vs[ i + 1 ];

    Eigen::Matrix<T, 6, 6> cov_inv = Eigen::Matrix<T, 6, 6>::Zero();
    //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
    //Eigen::MatrixXd cov_inv = cov.inverse();
    cov_inv.setIdentity();

    Eigen::Matrix<T, 6, 6> r_A = tmp_A.transpose() * cov_inv * tmp_A;
    Eigen::Matrix<T, 6, 1> r_b = tmp_A.transpose() * cov_inv * tmp_b;

    A.block<3, 3>( 0, 0 ) += r_A;
    b.segment<3>( 0 ) += r_b;
  }
  A = A * 1000.0;
  b = b * 1000.0;
  x = A.ldlt().solve( b );

  g = x.segment<3>( 0 );
  // LOG(INFO) << "g_norm_diff before refining: " <<fabs(g.norm() - g_norm);
  return fabs( g.norm() - g_norm ) < 0.5;
}


template <typename T>
void GravityEstimator<T>::RefineGravity(
    const std::deque<TransformAndPreintegrator<T>> &all_laser_transforms,
    const Eigen::Transform<T, 3, Eigen::Affine> &   transform_lb,
    const std::deque<Eigen::Matrix<T, 3, 1>> &      Vs,
    const T                                         k_g_norm,
    Eigen::Matrix<T, 3, 1> &                        g_approx )
{
  Eigen::Matrix<T, 3, 1> g0 = g_approx.normalized() * k_g_norm;

  // Laser in Base(IMU)
  Eigen::Matrix<T, 3, 1> tlb = transform_lb.translation();
  Eigen::Matrix<T, 3, 3> rlb = transform_lb.linear();

  Eigen::Matrix<T, 3, 1>              lx, ly;
  Eigen::Matrix<T, Eigen::Dynamic, 1> x;

  size_t window_size = all_laser_transforms.size();

  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A{ 2, 2 };
  A.setZero();
  Eigen::Matrix<T, Eigen::Dynamic, 1> b{ 2 };
  b.setZero();

  for ( int k = 0; k < 4; k++ )
  {
    Eigen::Matrix<T, 3, 2> lxly;
    lxly = TangentBasis( g0 );
    for ( size_t i = 0; i < window_size - 1; ++i )
    {
      const TransformAndPreintegrator<T> *frame_i = &all_laser_transforms[ i ];
      const TransformAndPreintegrator<T> *frame_j = &all_laser_transforms[ i + 1 ];
      if ( !frame_j->pre_integration )
      {
        continue;
      }
      T                      dt        = frame_j->pre_integration->deltaTij();
      Eigen::Matrix<T, 3, 3> frame_i_R = frame_i->transform.linear();
      Eigen::Matrix<T, 3, 3> frame_j_R = frame_j->transform.linear();
      Eigen::Matrix<T, 3, 1> frame_i_T = frame_i->transform.translation();
      Eigen::Matrix<T, 3, 1> frame_j_T = frame_j->transform.translation();

      Eigen::Matrix<T, 6, 2> tmp_A;
      tmp_A.setZero();
      Eigen::Matrix<T, 6, 1> tmp_b;
      tmp_b.setZero();

      tmp_A.block<3, 2>( 0, 0 ) = frame_i_R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>( 0, 0 ) = frame_j->pre_integration->deltaPij() + frame_i_R.transpose() * frame_j_R * tlb - tlb - frame_i_R.transpose() * dt * dt / 2 * g0 - frame_i_R.transpose() * ( frame_j_T - frame_i_T ) + dt * Eigen::Matrix3d::Identity() * Vs[ i ];

      tmp_A.block<3, 2>( 3, 0 ) = frame_i_R.transpose() * dt * Eigen::Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>( 3, 0 ) = frame_j->pre_integration->deltaVij() - frame_i_R.transpose() * dt * Eigen::Matrix3d::Identity() * g0 + Eigen::Matrix3d::Identity() * Vs[ i ] - frame_i_R.transpose() * frame_j_R * Vs[ i + 1 ];

      Eigen::Matrix<T, 6, 6> cov_inv = Eigen::Matrix<T, 6, 6>::Zero();
      //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
      //Eigen::MatrixXd cov_inv = cov.inverse();
      cov_inv.setIdentity();

      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> r_A = tmp_A.transpose() * cov_inv * tmp_A;
      Eigen::Matrix<T, Eigen::Dynamic, 1>              r_b = tmp_A.transpose() * cov_inv * tmp_b;

      A.block<2, 2>( 0, 0 ) += r_A;
      b.segment<2>( 0 ) += r_b;
    }
    A                                      = A * 1000.0;
    b                                      = b * 1000.0;
    x                                      = A.ldlt().solve( b );
    Eigen::Matrix<T, Eigen::Dynamic, 1> dg = x.segment<2>( 0 );
    g0                                     = ( g0 + lxly * dg ).normalized() * k_g_norm;
  }
  g_approx = g0;
  // for(int i = 0; i < window_size; ++i){
  //   Vs[i] = x.segment<3>(3 * i);
  // }
}

template <typename T>
bool GravityEstimator<T>::Estimate(
    const std::deque<TransformAndPreintegrator<T>> &all_laser_transforms,
    const Eigen::Transform<T, 3, Eigen::Affine> &   transform_lb,
    const std::deque<Eigen::Matrix<T, 3, 1>> &      Vs,
    T                                               g_norm,
    Eigen::Matrix<T, 3, 1> &                        g_approx )
{
  if ( !ApproximateGravity( all_laser_transforms, transform_lb, Vs, g_norm, g_approx ) )
  {
    return false;
  }

  RefineGravity( all_laser_transforms, transform_lb, Vs, g_norm, g_approx );

  double g_norm_diff = fabs( g_approx.norm() - g_norm );
  // LOG(INFO) << "g_norm_diff refined: " <<g_norm_diff;
  return g_norm_diff < 0.2;
};

}  // namespace lio_sam
