##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##
g++ -I../include -g -O0 --std=c++11 ../src/ahrs_driver.cpp ../src/lord_ahrs_driver.cpp ./test_lord_ahrs_driver.cpp -o test_lord_ahrs_driver
