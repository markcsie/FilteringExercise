#ifndef MEASUREMENT_MODEL_H
#define MEASUREMENT_MODEL_H

#include <memory>
#include <Eigen/Dense>

class MeasurementModel
{
public:
  typedef std::shared_ptr<MeasurementModel> Ptr;
  typedef std::shared_ptr<const MeasurementModel> ConstPtr;

  virtual ~MeasurementModel()
  {
  }

  // TODO: for non-linear model
//  virtual Eigen::VectorXd h(const Eigen::VectorXd &x) = 0;

  const std::string &getType() const
  {
    return type_;
  }

  const Eigen::MatrixXd &getQt() const
  {
    return Qt_;
  }

  const Eigen::MatrixXd &getQtInv() const
  {
    return Qt_inv_;
  }

protected:
  std::string type_;
  Eigen::MatrixXd Qt_;
  Eigen::MatrixXd Qt_inv_;
};

#endif // MEASUREMENT_MODEL_H
