#!/bin/bash
set -ex

# Embree Variables
EMBREE_TAG='emdag_working'
EMBREE_REPO='https://github.com/pshriwise/embree'
EMBREE_INSTALL_DIR=$HOME/EMBREE/

CURRENT_DIR=$(pwd)

# Embree Install
cd $HOME
mkdir EMBREE && cd EMBREE
git clone -b $EMBREE_TAG $EMBREE_REPO
mkdir build && cd build
cmake ../embree -DCMAKE_INSTALL_PREFIX=$EMBREE_INSTALL_DIR \
      -DEMBREE_ISPC_SUPPORT=OFF \
      -DEMBREE_TBB_ROOT=/usr
make -j2 && make -j2 install
rm -rf $HOME/EMBREE/embree

