#!/bin/bash
set -ex

# DAGMC Variables
DAGMC_BRANCH='dbldwn'
DAGMC_REPO='https://github.com/pshriwise/dagmc/'
DAGMC_INSTALL_DIR=$HOME/DAGMC/

CURRENT_DIR=$(pwd)

export LD_LIBRARY_PATH=$HOME/dd/lib:$LD_LIBRARY_PATH

# DAGMC Install
cd $HOME
mkdir DAGMC && cd DAGMC
git clone -b $DAGMC_BRANCH $DAGMC_REPO
mkdir build && cd build
cmake ../dagmc -DMOAB_DIR=$HOME/MOAB/ -DCMAKE_INSTALL_PREFIX=$HOME/DAGMC/
make -j && make -j test install
