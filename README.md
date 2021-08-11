# double-down : A double precision interface to Embree

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

[![CI Badge](https://github.com/pshriwise/double-down/workflows/Double%20Down%20CI/badge.svg)](https://github.com/pshriwise/double-down/actions?query=workflow%3A%22Double+Down+CI%22)


## Documentation

Double-down's documentation can be found [here](https://double-down.readthedocs.io/en/latest/).


## Summary

`double-down` is a double precision interface to Embree via the Mesh Oriented
dAtaBase (MOAB).

Geometric primitives (triangle elements) with double precision vertices are
stored in MOAB. Custom primitive types are defined in Embree as user-defined
geometry primitives.  These primitives interface with the MOAB instance to
provide robust bounding boxes and double precision intersection methods for
rays.

Ray values come in and out of the interface in double precision, making them
useful for scientific purposes while maintaining the performance provided by the
dedicated Intel developer team behind Embree.

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
