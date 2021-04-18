//
// Created by ksrdjan on 18.4.2021..
//

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "PID.h"
#include <limits>

class Controller
{
  static constexpr double kMax = std::numeric_limits<double>::max();

public:
  Controller();

  double RegulateSpeed(double reference, double input_speed);

  double RegulateCte(double cte);

private:
  void SIMC_parameterization(double &Kp,
                             double &Ti, double &Td, double tau_c, double k, double theta = 0, double tau_1 = kMax,
                             double tau_2 = kMax);

  PID pid_speed_;
  PID pid_steering_;
};


#endif //PID_CONTROLLER_H
