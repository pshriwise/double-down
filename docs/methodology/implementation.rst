.. _methodology_implementation:

==========================================================
Implementation of Mixed Precision Ray Tracing with Embree
==========================================================


To apply the mixed-precision approach with Embree, custom definitions of both
the underlying geometry primitives and ray definitions are required. First,
user-defined double precision primitives are generated with the triangles from
the Mesh Oriented datABase (`MOAB`_). With the understanding that the values of
the bounding boxes for each primitive would be interpreted by Embree in single
precision, the bounds are artificially extended enough to ensure that any
ray-box intersection that would have occurred in double precision would also
occur in single precision, but not enough to degrade performance of the BVH
traversal due to large overlaps of sibling bounding boxes. An :math:`\epsilon`
of 0.005 is sufficient to ensure robust ray intersections without significantly
impacting performance for models typically analyzed in Monte Carlo radiation
transport problems, the application for which this tool is designed.

.. _ray_dual:

.. figure:: ray_dual.png
    :alt: Extended Embree structures for mixed precision ray tracing.

To enable double precision ray definitions and intersection distances in and out
of the interface, a dual ray approach was employed. This approach extends the
:code:`RTCRay` and :code:`RTCHit` data structures in Embree to include double
precision duals of the ray origin, direction, and intersection distance (see
ray_dual_). During traversal, the single precision versions of the ray origin
and direction are used to traverse Embree's BVH. When a leaf node of the
BVH is reached, the primitives of the leaf are intersected in double
precision using the same Plücker intersection test [Platis2003_] that is
used in MOAB to ensure the same intersection distance (:code:`dtfar`)
and primitive normal (:code:`dNg_x, dNg_y, dNg_z`) are returned. The single
precision values for the intersection distance (:code:`tfar`) and primitive
normal (:code:`Ng_x, Ng_y, Ng_z`) are also updated so that occluded or
distant branches of the BVH will not be traversed. Values of the
:code:`primID` and :code:`geomID` are updated as usual as the intersected
primitive is updated. These two values are used to lookup the triangle and
surface hit in the MOAB mesh.

.. _MOAB: https://sigma.mcs.anl.gov/moab-library/
.. _Platis2003: https://www.tandfonline.com/doi/abs/10.1080/10867651.2003.10487593
