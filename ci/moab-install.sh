#!/bin/bash
set -ex

# MOAB Variables
MOAB_BRANCH='rh_separate'
MOAB_REPO='https://bitbucket.org/pshriwise/moab/'
MOAB_INSTALL_DIR=$HOME/MOAB/

CURRENT_DIR=$(pwd)

# MOAB Install
cd $HOME
mkdir MOAB && cd MOAB
git clone -b $MOAB_BRANCH $MOAB_REPO
mkdir build && cd build
cmake ../moab -DENABLE_HDF5=ON -DCMAKE_INSTALL_PREFIX=$MOAB_INSTALL_DIR
make -j && make -j test install
rm -rf $HOME/MOAB/moab
