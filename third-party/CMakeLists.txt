add_subdirectory(ParamHandler)
add_subdirectory(inih)
add_subdirectory(osqp)
add_subdirectory(JCQP)

if(STM32MP1_BUILD)
  # The JPos control path needs none of these. The MIT_Controller (MPC+WBC) needs
  # Goldfarb + qpOASES -- built only when STM32MP1_MIT is on, so it can't break the
  # working jpos/sim/stand build. EtherCAT/VectorNav/LORD stay off regardless.
  if(STM32MP1_MIT)
    add_subdirectory(Goldfarb_Optimizer)
    add_subdirectory(qpOASES)
  endif()
else()
  add_subdirectory(Goldfarb_Optimizer)
  add_subdirectory(qpOASES)
  add_subdirectory(lord_imu)
  add_subdirectory(SOEM)
  if(CMAKE_SYSTEM_NAME MATCHES Linux)
    add_subdirectory(vectornav)
  endif()
endif()
