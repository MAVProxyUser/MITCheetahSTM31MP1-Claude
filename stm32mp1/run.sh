#!/bin/bash
# Run the STM32MP1 Cheetah port (JPos controller). Run as root for RT scheduling.
# Executes from the package directory; jpos_ctrl finds its .so's via $ORIGIN rpath.
cd "$(dirname "$0")"
exec ./jpos_ctrl stm32mp1-defaults.yaml jpos-user-parameters.yaml
