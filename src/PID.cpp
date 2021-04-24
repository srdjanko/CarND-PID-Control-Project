#include "PID.h"
#include <cassert>
#include <limits>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() : e_s{0, 0}, deltaUd_s(0), u_s(0)
{}

PID::~PID()
{}

void PID::Init(double Kp_, double Ti_, double Td_, double Ts_, double alpha_,
               double sat_max_ = kMax,
               double sat_min_ = kMin)
{
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  assert((alpha_ >= 3.0 and alpha_ <= 20.0) and "Alpha parameter must be in range [3,20].");

  Kp = Kp_;
  Ti = Ti_;
  Td = Td_;
  Ts = Ts_;

  if (Td == 0)
  {
    A1 = 0;
    A2 = 0;
  }
  else
  {
    // Tustin discretization of the derivative action
    // alpha - The derivative attenuation parameter
    A1 = (2 * Td - alpha_ * Ts) / (2 * Td + alpha_ * Ts);
    A2 = 2 * Kp * alpha_ * Td / (2 * Td + alpha_ * Ts);
  }

  sat_max = sat_max_;
  sat_min = sat_min_;
}

void PID::Update(double e)
{
  /**
   * TODO: Update PID.
   */

  // Calculating incremental PID components
  auto delta_up = Kp * (e - e_s[0]);
  auto delta_ui = Kp * Ts / Ti * e;
  auto delta_ud = A1 * deltaUd_s + A2 * (e - 2 * e_s[0] + e_s[1]);

  // Final increment value
  auto delta_u = delta_up + delta_ui + delta_ud;

  // Refresh states
  e_s[1] = e_s[0], e_s[0] = e;
  deltaUd_s = delta_ud;
  IntegrateWithSaturation(delta_u);
}

double PID::GetOutput()
{
  /**
   * TODO: Calculate and return the total error
   */

  return u_s;
}

void PID::IntegrateWithSaturation(double delta_u)
{
  u_s = u_s + delta_u;

  // Anti - windup and saturation
  if (u_s >= sat_max)
  {
    u_s = sat_max;
  } else if (u_s <= sat_min)
  {
    u_s = sat_min;
  }
}
