// Minimal POD stand-in for the lcm-gen output (STM32MP1 port; core/driver use fields only).
#ifndef __spi_torque_t_hpp__
#define __spi_torque_t_hpp__

struct spi_torque_t {
  float tau_abad[4];
  float tau_hip[4];
  float tau_knee[4];
};

#endif
