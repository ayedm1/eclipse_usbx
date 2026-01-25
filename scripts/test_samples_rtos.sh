#!/bin/bash
CTEST_PARALLEL_LEVEL=4 $(dirname `realpath $0`)/../test/cmake/usbx/regression_demo/run.sh test $@
