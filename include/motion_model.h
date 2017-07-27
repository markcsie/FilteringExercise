#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <memory>
#include <Eigen/Dense>

class MotionModel
{
public:
  typedef std::shared_ptr<MotionModel> Ptr;
  typedef std::shared_ptr<const MotionModel> ConstPtr;


  virtual ~MotionModel()
  {
  }

  // TODO: for non-linear model
//  virtual Eigen::VectorXd g(const Eigen::VectorXd &u, const Eigen::VectorXd &x) const = 0;

  virtual const Eigen::MatrixXd &calculateRt() const
  {
    return Rt_;
  }

  const std::string &getType() const
  {
    return type_;
  }

protected:
  Eigen::MatrixXd Rt_;
  std::string type_;
};

#endif // MOTION_MODEL_H
