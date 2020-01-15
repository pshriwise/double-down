#!/bin/bash
set -ex

# DAGMC Variables
DAGMC_BRANCH='double_down_integration'
DAGMC_REPO='https://github.com/pshriwise/dagmc/'
DAGMC_INSTALL_DIR=$HOME/DAGMC/

CURRENT_DIR=$(pwd)

export LD_LIBRARY_PATH=$HOME/dd/lib:$LD_LIBRARY_PATH

# DAGMC Install
cd $HOME
mkdir DAGMC && cd DAGMC
git clone -b $DAGMC_BRANCH $DAGMC_REPO
mkdir build && cd build
cmake ../dagmc -DMOAB_DIR=$HOME/MOAB/ -DCMAKE_INSTALL_PREFIX=$HOME/DAGMC/ -DDOUBLE_DOWN=ON -DBUILD_STATIC_LIBS=OFF
make -j 3 install
make -j 3 ; cd ./src/dagmc/tests/; ./dagmc_simple_test
make -j 3 test
