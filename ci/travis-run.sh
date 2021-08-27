#!/bin/bash
set -ex

mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/dd/ \
         -DMOAB_DIR=$HOME/MOAB
make && make test && make install
