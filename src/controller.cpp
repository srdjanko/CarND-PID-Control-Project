//
// Created by ksrdjan on 18.4.2021..
//

#include "controller.h"
#include <algorithm>

namespace
{
  constexpr double kMax = std::numeric_limits<double>::max();
}

Controller::Controller()
{
  // Experimentally measured sample time
  auto Ts = 0.1;  // 100 ms

  // Setup CTE control
  // CTE model is derived from the kinetic bicycle model
  auto V = 20 * 1.6 / 3.6;  // Referent speed m/s, linearization point
  auto L = 5;               // Vehicle length m (assumed)
  auto kr = 8;
  auto k = V * V / L * kr;
  auto tau_c = 0.31;

  double Kp_cte, Ti_cte, Td_cte;
  SIMC_parameterization(Kp_cte, Ti_cte, Td_cte, tau_c, k);
  pid_steering_.Init(Kp_cte, Ti_cte, Td_cte, Ts, 3, 0.43, -0.43);

  // Setup speed control
  double Kp_spd = 0.3617, Ti_spd = 1.7241, Td_spd = 0;
  pid_speed_.Init(Kp_spd, Ti_spd, Td_spd, Ts, 3, 1, 0);
}

double Controller::RegulateSpeed(double reference, double input_speed)
{
  pid_speed_.Update(reference - input_speed);
  return pid_speed_.GetOutput();
}

double Controller::RegulateCte(double cte)
{
  pid_steering_.Update(cte);
  return pid_steering_.GetOutput();
}

void Controller::SIMC_parameterization(double &Kp,
                                       double &Ti, double &Td, double tau_c, double k,
                                       double theta, double tau_1, double tau_2)
{
  auto tau_d = std::min(tau_1, 4 * tau_c);
  auto tau_i = std::min(tau_2, 4 * tau_c);
  auto kc = (tau_1 == kMax ? 1 : tau_1) * (tau_2 == kMax ? 1 : tau_2) / (k * (tau_c + theta) * tau_d);

  Kp = kc * (tau_i + tau_d) / tau_i;
  Ti = tau_i + tau_d;
  Td = tau_d * tau_i / (tau_i + tau_d);
}
