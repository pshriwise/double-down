#!/bin/bash
set -ex

mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/dd/
make && make test && make install
