// Minimal POD stand-in for the lcm-gen output (STM32MP1 port; core/driver use fields only).
#ifndef __spi_data_t_hpp__
#define __spi_data_t_hpp__

struct spi_data_t {
  float q_abad[4];
  float q_hip[4];
  float q_knee[4];
  float qd_abad[4];
  float qd_hip[4];
  float qd_knee[4];
  int32_t flags[4];
  int32_t spi_driver_status;
};

#endif
