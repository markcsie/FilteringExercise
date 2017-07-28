#ifndef INCLUDE_KALMAN_FILTER_H_
#define INCLUDE_KALMAN_FILTER_H_

#include "./filter.h"
#include "./linear_motion_model.h"
#include "./linear_measurement_model.h"

class KalmanFilter : public Filter
{
 public:
  KalmanFilter(const Eigen::VectorXd &x0,
               const Eigen::MatrixXd &covariance0,
               LinearMotionModel::ConstPtr motion_model,
               LinearMeasurementModel::ConstPtr measurement_model);

  ~KalmanFilter() {}

  virtual void predict(const Eigen::VectorXd &u);
  virtual void correct(const Eigen::VectorXd &z);

  const Eigen::MatrixXd &getCovariance() const
  {
    return covariance_;
  }

 protected:
  Eigen::MatrixXd mean_;
  Eigen::MatrixXd covariance_;
};

#endif  // INCLUDE_KALMAN_FILTER_H_
