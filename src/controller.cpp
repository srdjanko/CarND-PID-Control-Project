//
// Created by ksrdjan on 18.4.2021..
//

#include "controller.h"

#include <algorithm>

namespace
{
  constexpr double max = std::numeric_limits<double>::max();
}

Controller::Controller()
{
  // Initialize the CTE PID
  auto Ts = 0.02;  // 20 ms

  // CTE model is derived from the kinetic bicycle model
  auto V = 100 / 3.6; // Referent speed m/s, linearization point
  auto L = 5;         // Vehicle length m (assumed)
  auto k_cte = V * V / L;
  auto tau_c_cte = 0.1;

  double Kp_cte, Ti_cte, Td_cte;
  SIMC_parameterization(Kp_cte, Ti_cte, Td_cte, tau_c_cte, k_cte);

  pid_speed_.Init(Kp_cte, Ti_cte, Td_cte, Ts, 3, 10, 10);

  // Initialize the speed PID, model is simple integrator (but what is the throttle?)
  auto tau_c_spd = 0.3;
  auto tau_1_spd = 0.05;
  auto k_spd = 1;

  double Kp_spd, Ti_spd, Td_spd;
  SIMC_parameterization(Kp_spd, Ti_spd, Td_spd, tau_c_spd, k_spd, tau_1_spd);

  pid_steering_.Init(Kp_spd, Ti_spd, Td_spd, Ts, 3, 1, -1);
}

double Controller::RegulateSpeed(double reference, double input_speed)
{
  pid_speed_.Update(reference, input_speed);
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
  auto kc = (tau_1 == max ? 1 : tau_1) * (tau_2 == max ? 1 : tau_2) / (k * (tau_c + theta) * tau_d);

  Kp = kc * (tau_i + tau_d) / tau_i;
  Ti = tau_i + tau_d;
  Td = tau_d * tau_i / (tau_i + tau_d);
}
