double-down : A double precision interface to Embree

[![CI Badge](https://github.com/pshriwise/double-down/workflows/Double%20Down%20CI/badge.svg)](https://github.com/pshriwise/double-down/actions?query=workflow%3A%22Double+Down+CI%22)


## Documentation

Please find the double-down documentation [here](https://double-down.readthedocs.io/en/latest/)


## Summary

`double-down` is a double precision interface to Embree via the Mesh Oriented dAtaBase (MOAB). Primitives (2D mesh elements) are stored in MOAB. New primitive types are defined using Embree's user geometry interface where they interace with the MOAB instance to provide robust bounding values for the mesh elements as well as intersection methods based in double precision. Ray values come in and out of the interface in double precision, making them useful for scientific purposes while applying the speed provided by the dedicated team of Intel developers who have created Embree.

## Installation

Assuming you have [MOAB](https://bitbucket.org/fathomteam/moab.git) and
[Embree](https://github.com/embree/embree) installed already then the
installation for double-down would be:


```
git clone https://github.com/pshriwise/double-down.git
cd double-down
mkdir build
cd build
cmake .. -DMOAB_DIR=/MOAB \
         -DCMAKE_INSTALL_PREFIX=.. \
         -DEMBREE_DIR=/embree
make
make install

```

## Other Notes

double-down is currently underdevelopment targeting Embree Version 3.6.1.

