.. Double-Down documentation master file, created by
   sphinx-quickstart on Fri Mar 26 13:29:08 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Double-Down:  A mixed-precision interface to Intel's Embree
===========================================================

Double-down is a mixed-precision interface to `Embree <https://www.embree.org/>`_ intended to enable particle tracking
for scientific applications relying on double precision. Currently, any mesh readable by
the `Mesh Oriented datABase (MOAB) <https://sigma.mcs.anl.gov/moab-library/>`_ and organized for
use with the `Direct Accelerated Geometry Monte Carlo (DAGMC) <https://svalinn.github.io/DAGMC/>`_ toolkit.

**Warning: Double-Down is under active development. The API is still undergoing changes.**

.. toctree::
   :maxdepth: 2
   :caption: Contents:

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Table of Contents
=================

.. toctree::
    :maxdepth: 2

    self
    api/index

Development Roadmap
===================

* Abstraction of the RayTracing interface and Triangle storage to be independent of MOAB.
* Support for additional mesh formats/libraries.