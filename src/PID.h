#ifndef PID_H
#define PID_H

#include <limits>

class PID
{

  static constexpr double kMax = std::numeric_limits<double>::max();
  static constexpr double kMin = std::numeric_limits<double>::min();

public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ti_, Td_, Ts_) The initial PID coefficients
   */
  void Init(double Kp_, double Ti_, double Td_, double Ts_, double alpha_, double sat_max_, double sat_min_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void Update(double e);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double GetOutput();

private:

  void IntegrateWithSaturation(double delta_u);

  // States
  double e_s[2];
  double deltaUd_s;
  double u_s;

  /**
   * PID Coefficients
   */
  double Kp;
  double Ti;
  double Td;
  double Ts;

  double A1;
  double A2;
  double sat_max;
  double sat_min;
};

#endif  // PID_H