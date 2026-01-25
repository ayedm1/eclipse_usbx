#!/bin/bash
CTEST_PARALLEL_LEVEL=4 $(dirname `realpath $0`)/../test/cmake/usbx/regression_samples_rtos/run.sh test $@
