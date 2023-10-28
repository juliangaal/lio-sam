#include "gravity_factor/gravityEstimator.h"

// namespace lio_sam
// {
// template <typename T>
// Eigen::MatrixXd GravityEstimator::TangentBasis( Eigen::Vector3d &g0 )
// {
//   Eigen::Vector3d b, c;
//   Eigen::Vector3d a = g0.normalized();
//   Eigen::Vector3d tmp( 0, 0, 1 );
//   if ( a == tmp )
//   {
//     tmp << 1, 0, 0;
//   }
//   b = ( tmp - a * ( a.transpose() * tmp ) ).normalized();
//   c = a.cross( b );
//   Eigen::MatrixXd bc( 3, 2 );
//   bc.block<3, 1>( 0, 0 ) = b;
//   bc.block<3, 1>( 0, 1 ) = c;
//   return bc;
// };

// template <typename T>
// bool GravityEstimator::Estimate(
//     const std::deque<TransformAndPreintegrator> &all_laser_transforms,
//     const Eigen::Affine3d &   transform_lb,
//     const std::deque<Eigen::Vector3d> &      Vs,
//     T                                               g_norm,
//     Eigen::Vector3d &                        g_approx )
// {
//   if ( !ApproximateGravity( all_laser_transforms, transform_lb, Vs, g_norm, g_approx ) )
//   {
//     return false;
//   }

//   RefineGravity( all_laser_transforms, transform_lb, Vs, g_norm, g_approx );

//   double g_norm_diff = fabs( g_approx.norm() - g_norm );
//   // LOG(INFO) << "g_norm_diff refined: " <<g_norm_diff;
//   return g_norm_diff < 0.2;
// };

// template <typename T>
// bool GravityEstimator::ApproximateGravity(
//     const std::deque<TransformAndPreintegrator> &all_laser_transforms,
//     const Eigen::Affine3d &   transform_lb,
//     const std::deque<Eigen::Vector3d> &      Vs,
//     const T                                         g_norm,
//     Eigen::Vector3d &                        g )
// {
//   size_t                              window_size = all_laser_transforms.size();
//   int                                 n_state     = window_size * 3 + 3;  //3g + n*3v
//   Eigen::VectorXd x;
//   // Laser in Base(IMU)
//   Eigen::Vector3d tlb = transform_lb.translation();
//   Eigen::Matrix3d rlb = transform_lb.linear();

//   Eigen::Matrix3d A;
//   A.setZero();
//   Eigen::Vector3d b;
//   b.setZero();

//   if ( window_size < 3 )
//   {
//     LOG( WARNING ) << ">>>>>>>Gravity estimate window size is not enough <<<<<<<";
//     return false;
//   }
//   for ( size_t i = 0; i < window_size - 1; ++i )
//   {
//     const TransformAndPreintegrator *frame_i = &all_laser_transforms[ i ];
//     const TransformAndPreintegrator *frame_j = &all_laser_transforms[ i + 1 ];

//     if ( !frame_j->pre_integration )
//     {
//       continue;
//     }

//     Eigen::MatrixXd tmp_A( 6, 3 );;
//     tmp_A.setZero();
//     Eigen::VectorXd tmp_b;
//     tmp_b.setZero();

//     T                      dt        = frame_j->pre_integration->deltaTij();
//     Eigen::Matrix3d frame_i_R = frame_i->transform.linear();
//     Eigen::Matrix3d frame_j_R = frame_j->transform.linear();
//     Eigen::Vector3d frame_i_T = frame_i->transform.translation();
//     Eigen::Vector3d frame_j_T = frame_j->transform.translation();

//     tmp_A.block<3, 3>( 0, 0 ) = frame_i_R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity();

//     tmp_b.block<3, 1>( 0, 0 ) = frame_j->pre_integration->deltaPij() + frame_i_R.transpose() * frame_j_R * tlb - tlb - frame_i_R.transpose() * ( frame_j_T - frame_i_T ) + dt * Eigen::Matrix3d::Identity() * Vs[ i ];

//     tmp_A.block<3, 3>( 3, 0 ) = frame_i_R.transpose() * dt * Eigen::Matrix3d::Identity();
//     tmp_b.block<3, 1>( 3, 0 ) = frame_j->pre_integration->deltaVij() + Eigen::Matrix3d::Identity() * Vs[ i ] - frame_i_R.transpose() * frame_j_R * Vs[ i + 1 ];

//     Eigen::Matrix<T, 6, 6> cov_inv = Eigen::Matrix<T, 6, 6>::Zero();
//     //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
//     //Eigen::MatrixXd cov_inv = cov.inverse();
//     cov_inv.setIdentity();

//     Eigen::Matrix<T, 6, 6> r_A = tmp_A.transpose() * cov_inv * tmp_A;
//     Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

//     A.block<3, 3>( 0, 0 ) += r_A;
//     b.segment<3>( 0 ) += r_b;
//   }
//   A = A * 1000.0;
//   b = b * 1000.0;
//   x = A.ldlt().solve( b );

//   g = x.segment<3>( 0 );
//   // LOG(INFO) << "g_norm_diff before refining: " <<fabs(g.norm() - g_norm);
//   return fabs( g.norm() - g_norm ) < 0.5;
// }


// template <typename T>
// void GravityEstimator::RefineGravity(
//     const std::deque<TransformAndPreintegrator> &all_laser_transforms,
//     const Eigen::Affine3d &   transform_lb,
//     const std::deque<Eigen::Vector3d> &      Vs,
//     const T                                         k_g_norm,
//     Eigen::Vector3d &                        g_approx )
// {
//   Eigen::Vector3d g0 = g_approx.normalized() * k_g_norm;

//   // Laser in Base(IMU)
//   Eigen::Vector3d tlb = transform_lb.translation();
//   Eigen::Matrix3d rlb = transform_lb.linear();

//   Eigen::Vector3d              lx, ly;
//   Eigen::VectorXd x;

//   size_t window_size = all_laser_transforms.size();

//   Eigen::MatrixXd A{ 2, 2 };
//   A.setZero();
//   Eigen::VectorXd b{ 2 };
//   b.setZero();

//   for ( int k = 0; k < 4; k++ )
//   {
//     Eigen::MatrixXd lxly;
//     lxly = TangentBasis( g0 );
//     for ( size_t i = 0; i < window_size - 1; ++i )
//     {
//       const TransformAndPreintegrator *frame_i = &all_laser_transforms[ i ];
//       const TransformAndPreintegrator *frame_j = &all_laser_transforms[ i + 1 ];
//       if ( !frame_j->pre_integration )
//       {
//         continue;
//       }
//       T                      dt        = frame_j->pre_integration->deltaTij();
//       Eigen::Matrix3d frame_i_R = frame_i->transform.linear();
//       Eigen::Matrix3d frame_j_R = frame_j->transform.linear();
//       Eigen::Vector3d frame_i_T = frame_i->transform.translation();
//       Eigen::Vector3d frame_j_T = frame_j->transform.translation();

//       Eigen::MatrixXd tmp_A;
//       tmp_A.setZero();
//       Eigen::VectorXd tmp_b;
//       tmp_b.setZero();

//       tmp_A.block<3, 2>( 0, 0 ) = frame_i_R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity() * lxly;
//       tmp_b.block<3, 1>( 0, 0 ) = frame_j->pre_integration->deltaPij() + frame_i_R.transpose() * frame_j_R * tlb - tlb - frame_i_R.transpose() * dt * dt / 2 * g0 - frame_i_R.transpose() * ( frame_j_T - frame_i_T ) + dt * Eigen::Matrix3d::Identity() * Vs[ i ];

//       tmp_A.block<3, 2>( 3, 0 ) = frame_i_R.transpose() * dt * Eigen::Matrix3d::Identity() * lxly;
//       tmp_b.block<3, 1>( 3, 0 ) = frame_j->pre_integration->deltaVij() - frame_i_R.transpose() * dt * Eigen::Matrix3d::Identity() * g0 + Eigen::Matrix3d::Identity() * Vs[ i ] - frame_i_R.transpose() * frame_j_R * Vs[ i + 1 ];

//       Eigen::Matrix<T, 6, 6> cov_inv = Eigen::Matrix<T, 6, 6>::Zero();
//       //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
//       //Eigen::MatrixXd cov_inv = cov.inverse();
//       cov_inv.setIdentity();

//       Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
//       Eigen::VectorXd              r_b = tmp_A.transpose() * cov_inv * tmp_b;

//       A.block<2, 2>( 0, 0 ) += r_A;
//       b.segment<2>( 0 ) += r_b;
//     }
//     A                                      = A * 1000.0;
//     b                                      = b * 1000.0;
//     x                                      = A.ldlt().solve( b );
//     Eigen::VectorXd dg = x.segment<2>( 0 );
//     g0                                     = ( g0 + lxly * dg ).normalized() * k_g_norm;
//   }
//   g_approx = g0;
//   // for(int i = 0; i < window_size; ++i){
//   //   Vs[i] = x.segment<3>(3 * i);
//   // }
// }


// }  // namespace lio_sam


namespace lio_sam
{
Eigen::MatrixXd GravityEstimator::TangentBasis( Eigen::Vector3d &g0 )
{
  Eigen::Vector3d b, c;
  Eigen::Vector3d a = g0.normalized();
  Eigen::Vector3d tmp( 0, 0, 1 );
  if ( a == tmp )
  {
    tmp << 1, 0, 0;
  }
  b = ( tmp - a * ( a.transpose() * tmp ) ).normalized();
  c = a.cross( b );
  Eigen::MatrixXd bc( 3, 2 );
  bc.block<3, 1>( 0, 0 ) = b;
  bc.block<3, 1>( 0, 1 ) = c;
  return bc;
};

bool GravityEstimator::Estimate(
    const std::deque<TransformAndPreintegrator> &all_laser_transforms,
    const Eigen::Affine3d &                      transform_lb,
    const std::deque<Eigen::Vector3d> &          Vs,
    const double                                 g_norm,
    Eigen::Vector3d &                            g_approx )
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


bool GravityEstimator::ApproximateGravity(
    const std::deque<TransformAndPreintegrator> &all_laser_transforms,
    const Eigen::Affine3d &                      transform_lb,
    const std::deque<Eigen::Vector3d> &          Vs,
    const double                                 g_norm,
    Eigen::Vector3d &                            g )
{
  size_t window_size = all_laser_transforms.size();
  // int             n_state     = window_size * 3 + 3;  //3g + n*3v
  Eigen::VectorXd x;
  // Laser in Base(IMU)
  Eigen::Vector3d tlb = transform_lb.translation();
  // Eigen::Matrix3d rlb = transform_lb.linear();

  Eigen::Matrix3d A{ 3, 3 };
  A.setZero();
  Eigen::Vector3d b{ 3 };
  b.setZero();

  if ( window_size < 3 )
  {
    LOG( WARNING ) << ">>>>>>>Gravity estimate window size is not enough <<<<<<<";
    return false;
  }
  for ( size_t i = 0; i < window_size - 1; ++i )
  {
    const TransformAndPreintegrator *frame_i = &all_laser_transforms[ i ];
    const TransformAndPreintegrator *frame_j = &all_laser_transforms[ i + 1 ];

    if ( !frame_j->pre_integration )
    {
      continue;
    }

    Eigen::MatrixXd tmp_A( 6, 3 );
    tmp_A.setZero();
    Eigen::VectorXd tmp_b( 6 );
    tmp_b.setZero();

    double          dt        = frame_j->pre_integration->deltaTij();
    Eigen::Matrix3d frame_i_R = frame_i->transform.linear();
    Eigen::Matrix3d frame_j_R = frame_j->transform.linear();
    Eigen::Vector3d frame_i_T = frame_i->transform.translation();
    Eigen::Vector3d frame_j_T = frame_j->transform.translation();

    tmp_A.block<3, 3>( 0, 0 ) = frame_i_R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity();

    tmp_b.block<3, 1>( 0, 0 ) = frame_j->pre_integration->deltaPij() + frame_i_R.transpose() * frame_j_R * tlb - tlb - frame_i_R.transpose() * ( frame_j_T - frame_i_T ) + dt * Eigen::Matrix3d::Identity() * Vs[ i ];

    tmp_A.block<3, 3>( 3, 0 ) = frame_i_R.transpose() * dt * Eigen::Matrix3d::Identity();
    tmp_b.block<3, 1>( 3, 0 ) = frame_j->pre_integration->deltaVij() + Eigen::Matrix3d::Identity() * Vs[ i ] - frame_i_R.transpose() * frame_j_R * Vs[ i + 1 ];

    Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
    //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
    //Eigen::MatrixXd cov_inv = cov.inverse();
    cov_inv.setIdentity();

    Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
    Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

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


void GravityEstimator::RefineGravity(
    const std::deque<TransformAndPreintegrator> &all_laser_transforms,
    const Eigen::Affine3d &                      transform_lb,
    const std::deque<Eigen::Vector3d> &          Vs,
    const double                                 k_g_norm,
    Eigen::Vector3d &                            g_approx )
{
  Eigen::Vector3d g0 = g_approx.normalized() * k_g_norm;

  // Laser in Base(IMU)
  Eigen::Vector3d tlb = transform_lb.translation();
  // Eigen::Matrix3d rlb = transform_lb.linear();

  Eigen::Vector3d lx, ly;
  Eigen::VectorXd x;

  size_t window_size = all_laser_transforms.size();

  Eigen::MatrixXd A{ 2, 2 };
  A.setZero();
  Eigen::VectorXd b{ 2 };
  b.setZero();

  for ( int k = 0; k < 4; k++ )
  {
    Eigen::MatrixXd lxly;
    lxly = TangentBasis( g0 );
    for ( size_t i = 0; i < window_size - 1; ++i )
    {
      const TransformAndPreintegrator *frame_i = &all_laser_transforms[ i ];
      const TransformAndPreintegrator *frame_j = &all_laser_transforms[ i + 1 ];
      if ( !frame_j->pre_integration )
      {
        continue;
      }
      double          dt        = frame_j->pre_integration->deltaTij();
      Eigen::Matrix3d frame_i_R = frame_i->transform.linear();
      Eigen::Matrix3d frame_j_R = frame_j->transform.linear();
      Eigen::Vector3d frame_i_T = frame_i->transform.translation();
      Eigen::Vector3d frame_j_T = frame_j->transform.translation();

      Eigen::MatrixXd tmp_A( 6, 2 );
      tmp_A.setZero();
      Eigen::VectorXd tmp_b( 6 );
      tmp_b.setZero();

      tmp_A.block<3, 2>( 0, 0 ) = frame_i_R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>( 0, 0 ) = frame_j->pre_integration->deltaPij() + frame_i_R.transpose() * frame_j_R * tlb - tlb - frame_i_R.transpose() * dt * dt / 2 * g0 - frame_i_R.transpose() * ( frame_j_T - frame_i_T ) + dt * Eigen::Matrix3d::Identity() * Vs[ i ];

      tmp_A.block<3, 2>( 3, 0 ) = frame_i_R.transpose() * dt * Eigen::Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>( 3, 0 ) = frame_j->pre_integration->deltaVij() - frame_i_R.transpose() * dt * Eigen::Matrix3d::Identity() * g0 + Eigen::Matrix3d::Identity() * Vs[ i ] - frame_i_R.transpose() * frame_j_R * Vs[ i + 1 ];

      Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
      //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
      //Eigen::MatrixXd cov_inv = cov.inverse();
      cov_inv.setIdentity();

      Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
      Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

      A.block<2, 2>( 0, 0 ) += r_A;
      b.segment<2>( 0 ) += r_b;
    }
    A                  = A * 1000.0;
    b                  = b * 1000.0;
    x                  = A.ldlt().solve( b );
    Eigen::VectorXd dg = x.segment<2>( 0 );
    g0                 = ( g0 + lxly * dg ).normalized() * k_g_norm;
  }
  g_approx = g0;
  // for(int i = 0; i < window_size; ++i){
  //   Vs[i] = x.segment<3>(3 * i);
  // }
}
}  // namespace lio_sam
