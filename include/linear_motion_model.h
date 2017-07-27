#ifndef LINEARMOTIONMODEL_H
#define LINEARMOTIONMODEL_H

#include "motion_model.h"
#include <Eigen/Dense>

class LinearMotionModel : public MotionModel
{
public:
  typedef std::shared_ptr<LinearMotionModel> Ptr;
  typedef std::shared_ptr<const LinearMotionModel> ConstPtr;

  LinearMotionModel(const Eigen::MatrixXd &A,
                    const Eigen::MatrixXd &B,
                    const Eigen::MatrixXd &Rt) : A_(A), B_(B)
  {
    Rt_ = Rt;
  }

  virtual ~LinearMotionModel()
  {
  }

  const Eigen::MatrixXd &getA() const
  {
    return A_;
  }

  const Eigen::MatrixXd &getB() const
  {
    return B_;
  }

protected:
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;

};

#endif // LINEARMOTIONMODEL_H
