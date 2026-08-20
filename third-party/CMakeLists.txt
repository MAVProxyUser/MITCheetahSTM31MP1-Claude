add_subdirectory(ParamHandler)
add_subdirectory(inih)
add_subdirectory(osqp)
add_subdirectory(JCQP)

if(STM32MP1_BUILD)
  # The Unitree-RS485 / MPU-9150 port builds only the JPos control path, which needs
  # none of these: Goldfarb_Optimizer (test + MIT_Controller WBC only; its Array.hh
  # also trips gcc-15 template checks), qpOASES (MIT_Controller), EtherCAT (SOEM),
  # the VectorNav or LORD IMUs. Re-enable when porting the full MIT_Controller.
else()
  add_subdirectory(Goldfarb_Optimizer)
  add_subdirectory(qpOASES)
  add_subdirectory(lord_imu)
  add_subdirectory(SOEM)
  if(CMAKE_SYSTEM_NAME MATCHES Linux)
    add_subdirectory(vectornav)
  endif()
endif()
