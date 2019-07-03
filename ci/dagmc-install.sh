#!/bin/bash
set -ex

# DAGMC Variables
DAGMC_BRANCH='dbldwn'
DAGMC_REPO='https://github.com/pshriwise/dagmc/'
DAGMC_INSTALL_DIR=$HOME/DAGMC/

CURRENT_DIR=$(pwd)

# DAGMC Install
cd $HOME
mkdir DAGMC && cd DAGMC
git clone -b $DAGMC_BRANCH $DAGMC_REPO
mkdir build && cd build
cmake ../dagmc -DMOAB_DIR=$HOME/MOAB/
make -j && make -j test install
