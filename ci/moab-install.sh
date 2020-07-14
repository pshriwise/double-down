#!/bin/bash
set -ex

# MOAB Variables
MOAB_BRANCH='master'
MOAB_REPO='https://bitbucket.org/fathomteam/moab/'
MOAB_INSTALL_DIR=$HOME/MOAB/

CURRENT_DIR=$(pwd)

# MOAB Install
cd $HOME
mkdir MOAB && cd MOAB
git clone -b $MOAB_BRANCH $MOAB_REPO
mkdir build && cd build
cmake ../moab -DENABLE_HDF5=ON -DCMAKE_INSTALL_PREFIX=$MOAB_INSTALL_DIR -DENABLE_BLASLAPACK=OFF
make -j 3 && make -j 3 test install
cmake ../moab -DBUILD_SHARED_LIBS=OFF
make -j 3 install
rm -rf $HOME/MOAB/moab
