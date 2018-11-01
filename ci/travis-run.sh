#!/bin/bash
set -ex

mkdir $HOME/opt/ && cd $HOME/opt/
git clone https://github.com/pshriwise/double-down $DDHOME
mkdir $DDBUILD && cd $DDBUILD
cmake $DDHOME
make && make test

