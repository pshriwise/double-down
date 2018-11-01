#!/bin/bash
set -ex

# Embree Variables
EMBREE_TAG='emdag_working'
EMBREE_REPO='https://github.com/pshriwise/embree'
EMBREE_INSTALL_DIR=$HOME/EMBREE/

CURRENT_DIR=$(pwd)

# Embree Install
# MOAB Install
cd $HOME
mkdir EMBREE && cd EMBREE
git clone -b $EMBREE_TAG $EMBREE_REPO
mkdir build && cd build
cmake ../moab -DCMAKE_INSTALL_PREFIX=$EMBREE_INSTALL_DIR \
      -DEMBREE_ISPC_SUPPORT=OFF \
      -DEMBREE_TBB_ROOT=/usr
make -j && make -j test install
rm -rf $HOME/EMBREE/moab
export LD_LIBRARY_PATH=$EMBREE_INSTALL_DIR/lib:$LD_LIBRARY_PATH
