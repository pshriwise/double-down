#!/bin/bash
set -ex


git clone https://github.com/pshriwise/double-down $DDHOME
cd $DDBUILD
cmake $DDHOME
make

