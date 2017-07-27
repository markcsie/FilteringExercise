#ifndef FILTER_H
#define FILTER_H

#include <Eigen/Dense>

#include "motion_model.h"
#include "measurement_model.h"

class Filter
{
public:
  virtual ~Filter()
  {
  }

  virtual void predict(const Eigen::VectorXd &u) = 0;
  virtual void correct(const Eigen::VectorXd &z) = 0;

  const std::string &getType() const
  {
    return type_;
  }

  const Eigen::VectorXd &getX() const
  {
    return x_;
  }

protected:
  std::string type_;
  Eigen::VectorXd x_;

  MotionModel::ConstPtr motion_model_;
  MeasurementModel::ConstPtr measurement_model_;
};

#endif // FILTER_H
