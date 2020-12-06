#!/bin/bash
set -ex

# DAGMC Variables
DAGMC_BRANCH='develop'
DAGMC_REPO='https://github.com/pshriwise/dagmc/'
DAGMC_INSTALL_DIR=$HOME/DAGMC/

CURRENT_DIR=$(pwd)

# DAGMC Install
cd $HOME
mkdir DAGMC && cd DAGMC
git clone -b $DAGMC_BRANCH $DAGMC_REPO
mkdir build && cd build
cmake ../dagmc -DCMAKE_INSTALL_PREFIX=$HOME/DAGMC/ \
      -DMOAB_DIR=$HOME/MOAB/ \
      -DDOUBLE_DOWN=ON \
      -DCMAKE_PREFIX_PATH=$HOME/dd/lib \
      -DBUILD_STATIC_LIBS=OFF
make -j 3 install
make -j 3 ; cd ./src/dagmc/tests/; ./dagmc_simple_test
make -j 3 test
