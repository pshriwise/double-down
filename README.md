double-down : A double precision interface to Embree

[![CI Badge](https://github.com/pshriwise/double-down/workflows/Double%20Down%20CI/badge.svg)](https://github.com/pshriwise/double-down/actions?query=workflow%3A%22Double+Down+CI%22)


## Summary

`double-down` is a double precision interface to Embree via the Mesh Oriented dAtaBase (MOAB). Primitives (2D mesh elements) are stored in MOAB. New primitive types are defined using Embree's user geometry interface where they interace with the MOAB instance to provide robust bounding values for the mesh elements as well as intersection methods based in double precision. Ray values come in and out of the interface in double precision, making them useful for scientific purposes while applying the speed provided by the dedicated team of Intel developers who have created Embree.


## Other Notes

double-down is currently underdevelopment targeting Embree Version 3.6.1.
