#ifndef DD_RTI_H
#define DD_RTI_H

#include "embree3/rtcore.h"
#include "embree3/rtcore_ray.h"

#include "moab/Core.hpp"
#include "moab/GeomQueryTool.hpp"
#include "moab/GeomTopoTool.hpp"

#include "primitives.hpp"
#include "MOABRay.h"
#include "MOABDirectAccess.h"

#include <unordered_map>
#include <memory>

class Node;

/*! Interface for constructing BVH's, firing rays, and performing point containment checks. */
class RayTracingInterface {

  /*! This class is used to manage storage of Dbltri objects for each volume. */
  class DblTriStorage {
  public:

    //! \brief Determine if a volume has DblTri objects being stored.
    bool is_storing(moab::EntityHandle vol)
    {
      return storage_.find(vol) != storage_.end();
    }

    //! \brief Store a set of DblTris for a volume.
    void store(moab::EntityHandle vol, std::vector<DblTri>&& buffer)
    {
      if (storage_.find(vol) != storage_.end()) { return; }
      storage_[vol] = buffer;
    }

    //! \brief Retrieve the DblTri's for the specified volume.
    std::vector<DblTri>& retrieve_buffer(moab::EntityHandle vol)
    {
      return storage_.at(vol);
    }

    //! \brief Retrieve the DblTri's for the specified volume.
    const std::vector<DblTri>& retrieve_buffer(moab::EntityHandle vol) const
    {
      return storage_.at(vol);
    }

    //! \brief Remove the storage for the specified volume.
    void free_storage(moab::EntityHandle vol)
    {
      if (is_storing(vol)) storage_.erase(vol);
    }

    //! \brief Clear all storage.
    void clear()
    {
      storage_.clear();
    }

  private:
    std::unordered_map<moab::EntityHandle, std::vector<DblTri>> storage_; //!< Containmer mapping a volume handle to its DblTri's.
  };

  public:
  //! \brief Constructor taking a MOAB interface pointer. A GeomTopoTool will be created
  //! internally for this instance.
  //! \param mbi Pointer to a moab::Interface.
  RayTracingInterface(moab::Interface* mbi);

  //! \brief Constructor taking a GeomTopoTool shared pointer.
  //! \param gtt Shared pointer to a moab::GeomTopoTool object.
  RayTracingInterface(std::shared_ptr<moab::GeomTopoTool> gtt);

  //! \brief Default constructor, both a MOAB instance and GeomTopoTool object will be created internally.
  RayTracingInterface() : MBI(new moab::Core()), GTT(std::make_shared<moab::GeomTopoTool>(MBI)) {  }

  //! \brief Destructor. Clears all storage and removes all Embree scenes and geometries.
  ~RayTracingInterface() {
                            shutdown();
                            buffer_storage.clear();
                         }

  // Public Functions
  //! \brief Load a file to be used for the RTI.
  //! \param filename Path to the mesh file to load.
  moab::ErrorCode load_file(std::string filename);

  //! \brief Initialize the RTI, building acceleration datastructures and internal storage.
  //! Assumes that the MOAB file is already open.
  moab::ErrorCode init();

  //! \brief Release all Embree scenes and device.
  void shutdown();

  //! \brief Check location \p xyz for containment in the specified \p volume.
  //! Performs a point containment query by firing a single ray and checking the dot product
  //! of the ray direction and the sense-adjusted normal of the triangle hit. Falls back onto
  //! RayTracingInterface::point_in_volume_slow() in ambiguous situations.
  //! \param volume MOAB EntityHandle of the volume to check for containment.
  //! \param xyz Location to check.
  //! \param result Result of the query (1 if inside, 0 if outside).
  //! \param uvw Ray direction to use in the query (randomly generated if one is not provided).
  //! \param history RayHistory object, used to ?? if provided.
  //! \param overlap_tol Maximum distance tolerated for self-overlapping volumes.
  moab::ErrorCode point_in_volume(const moab::EntityHandle volume,
                                  const double xyz[3],
                                  int& result,
                                  const double *uvw,
                                  const moab::GeomQueryTool::RayHistory *history,
                                  double overlap_tol = 0.0);

  //! \brief A slower, robust method for point containment of location \p xyz in \p volume.
  //! This method fires infinitely long rays in opposing directions and collects all
  //! intersections along the ray. The number of exiting/entering intersections are
  //! used to determine the points containment in the volume.
  //! \param volume MOAB EntityHandle of the volume to check for containment.
  //! \param xyz Location to check.
  //! \param result Result of the query (1 if inside, 0 if outside).
  moab::ErrorCode point_in_volume_slow(moab::EntityHandle volume,
                                       const std::array<double, 3> xyz,
                                       int& result);

  //! \brief Calculates the solid angle of a polygon, \p face, with respect to \p point.
  //! This method is adapted from "Point in Polyhedron Testing Using Spherical Polygons", Paulo Cezar
  //! Pinto Carvalho and Paulo Roma Cavalcanti, _Graphics Gems V_, pg. 42.  Original algorithm
  //! was described in "An Efficient Point In Polyhedron Algorithm", Jeff Lane, Bob Magedson,
  //! and Mike Rarick, _Computer Vision, Graphics, and Image Processing 26_, pg. 118-225, 1984.
  //! \param face MOAB EntityHandle for the face.
  //! \param point MOAB CartVect of the location to use fo the solid angle.
  //! \param solid_angle Solid angle of the polygon, \p face, with respect to \p point.
  moab::ErrorCode poly_solid_angle( moab::EntityHandle face,
                                    const moab::CartVect& point,
                                    double& solid_angle);

  void boundary_case(moab::EntityHandle volume,
                     int& result,
                     double u,
                     double v,
                     double w,
                     moab::EntityHandle facet,
                     moab::EntityHandle surface);


  moab::ErrorCode test_volume_boundary(const moab::EntityHandle volume,
                                       const moab::EntityHandle surface,
                                       const double xyz[3],
                                       const double uvw[3],
                                       int& result,
                                       const moab::GeomQueryTool::RayHistory* history = 0);


  //! \brief Fire ray with origin \p point, unit direction \dir, and length \p user_dist_limit.
  //! Fires a ray, returning the distance to the nearest intersection (\p next_surf_dist)
  //! and the surface hit (\p next_surf). Any triangles already present in a provided
  //! \p history are ignored when searching for new hits. Any new triangles hit will be
  //! appended to the \p history if provided.
  //! \param volume MOAB EntityHandle of the volume to fire on.
  //! \param point x,y,z coordinates of the ray origin.
  //! \param dir u,v,w values of the unit ray direction.
  //! \param next_surf Set to the MOAB EntityHandle of the hit surface.
  //! \param next_surf_dist Set to the distance of the next surface along the ray.
  //! \param history MOAB RayHistory object, used to ignore specific triangles and track newly intersected triangles.
  //! \param user_dist_limit Length of the ray specified by the user.  A near-infinite value is used by default.
  //! \param ray_orientation Indicator of whether hits should be counted for aligned or opposing sense-adjusted triangle
  //! dot product check. Default is aligned normals.
  //! \param dum Placeholder for compatibility with MOAB's GeomQueryTool::ray_fire. (ignore)
  moab::ErrorCode ray_fire(const moab::EntityHandle volume,
                           const double point[3],
                           const double dir[3],
                           moab::EntityHandle& next_surf,
                           double& next_surf_dist,
                           moab::GeomQueryTool::RayHistory* history = 0,
                           double user_dist_limit = 0,
                           int ray_orientation = 1,
                           void* dum=NULL);

  //! \brief Get the oriented bounding box for the specified \p volume.
  // Note: this will always return an axis-aligned bounding box as oriented boxes are not used in double-down.
  //! \param volume MOAB EntityHandle of the volume.
  //! \param center Coordinates for the center of the bounding box.
  //! \param axis0 Scaled vector from the center to the edge of the box along axis 0. (Always along x-axis)
  //! \param axis1 Scaled vector from the center to the edge of the box along axis 1. (Always along x-axis)
  //! \param axis2 Scaled vector from the center to the edge of the box along axis 2. (Always along x-axis)
  moab::ErrorCode get_obb(moab::EntityHandle volume,
                          std::array<double, 3>& center,
                          std::array<double, 3>& axis0,
                          std::array<double, 3>& axis1,
                          std::array<double, 3>& axis2);

  //! \brief Get an axis-aligned bounding bos for the specified \p volume.
  //! \param volume MOAB EntityHandle of the volume.
  //! \param llc x,y,z coordinates for the lower left corner of the box.
  //! \param urc x,y,z coordinates for the upper right corner of the box.
  moab::ErrorCode get_bbox(moab::EntityHandle volume,
                           std::array<double, 3>& llc,
                           std::array<double, 3>& urc);

  //! \brief Get a MOAB Range of all the volumes in the RayTracingInterface.
  //! \param vols Set to the range of volumes in the RayTracingInterface.
  moab::ErrorCode get_vols(moab::Range& vols);

  //! \brief Wrapper call for firing a ray in Embree. Separate from the
  //! calls made in RayTracingInterface::ray_fire.
  //! \param volume MOAB EntityHandle of the volume.
  void fire(moab::EntityHandle volume, RTCDRayHit &rayhit);

  //! \brief Allocates space for triangle reference information for the volume
  //! \param volume MOAB EntityHandle of the volume.
  moab::ErrorCode
  allocateTriangleBuffer(moab::EntityHandle volume);

  //! \brief Creates the BVH for the specified volume.
  //! In addition to creating the BVH using Embree, this function ensures
  //! that DblTri's connecting the MOAB triangles to the user-defined
  //! geometry in Embree are allocated and stored for the volume.
  //! \param volume MOAB EntityHandle of the volume.
  moab::ErrorCode
  createBVH(moab::EntityHandle volume);

  //! \brief Deletes the BVH for the volume if present.
  //! \param volume MOAB EntityHandle of the volume.
  void
  deleteBVH(moab::EntityHandle volume);

  //! \brief Finds the distance to the closest point on the \p volume
  //! from a specified location, \p point.
  //! \param volume MOAB EntityHandle of the volume.
  //! \param point x,y,z coordinates of the query location.
  //! \param result Set to the distance of the nearest point on the \p volume.
  //! \param closest_surf If provided, set to the MOAB EntityHandle of the nearest surface to \p point.
  moab::ErrorCode
  closest_to_location(moab::EntityHandle volume,
                      const double point[3],
                      double & result,
                      moab::EntityHandle* closest_surf = 0);

  //! \brief Internal function for getting the closest intersection to \p loc on the \p volume.
  //! \param volume MOAB EntityHandle of the volume.
  //! \param loc x,y,z coordinates of the query location.
  //! \param result Set to the distance of the nearest location on the \p volume.
  //! \param surface If present, set to the MOAB EntityHandle of the nearest surface.
  //! \param facet If present, set to the MOAB EntityHandle of the triangle containing the nearest point.
  void closest(moab::EntityHandle volume,
               const double loc[3],
               double &result,
               moab::EntityHandle* surface = 0,
               moab::EntityHandle* facet = 0);

  //! \brief Finds the normal of the nearest location on the provided \p surface.
  //! \param surface MOAB EntityHandle of the surface.
  //! \param loc x,y,z coordinates of the query location.
  //! \param angle u,v,w components of the direction unit vector.
  //! \param history If provided, the last facet in the history is used to determine the normal.
  moab::ErrorCode get_normal(moab::EntityHandle surface,
                             const double loc[3],
                             double angle[3],
                             const moab::GeomQueryTool::RayHistory* history = 0);

  //! \brief Returns the volume of the specified \p volume.
  //! \param volume MOAB EntityHandle of the volume.
  //! \param result Set to the size of the volume.
  moab::ErrorCode
  measure_volume(moab::EntityHandle volume,
                 double& result);

  //! \brief Returns the surface of the specified \p surface.
  //! \param surface MOAB EntityHandle of the surface.
  //! \param result Set to the size of the surface.
  moab::ErrorCode
  measure_area(moab::EntityHandle surface,
               double& result);

  //! \brief Indicates whether or not acceleration data structures are present.
  bool
  has_bvh() const;

  // Inline functions

  //! \brief Raw pointer to the MOAB GeomTopoTool.
  inline
  moab::GeomTopoTool* gttool() { return GTT.get(); }

  //! \brief Accessor for the numerical precision.
  inline
  double get_numerical_precision() { return numerical_precision; }

  //! \brief Accessor fo the allowed overlap thickness of self-intersecting volumes.
  inline
  double get_overlap_thickness() { return overlap_thickness; }

  //! \brief Sets the value of numerical precision.
  //! \param val New value for the numerical precision.
  inline
  void set_numerical_precision(double val) { numerical_precision = val; }

  //! \brief Sets the value of overlap thickness.
  //! \param val New value for the overlap thickness.
  inline
  void set_overlap_thickness(double val) { overlap_thickness = val; }

  //! \brief Accessor for the MBDirectAccess object.
  inline std::shared_ptr<MBDirectAccess> direct_access_manager() { return mdam; }

  // Member variables
  private:
  moab::Interface* MBI; //!< Underlying MOAB instance pointer.
  std::shared_ptr<moab::GeomTopoTool> GTT; //!< MOAB GeomTopoTool instance.
  std::shared_ptr<MBDirectAccess> mdam; //!< MBDirectAccess instance.
  DblTriStorage buffer_storage; //!< Per-volume storage for DblTri instances
  std::unordered_map<moab::EntityHandle, RTCScene> scene_map; //!< Mapping from MOAB volume EntityHandle's to Embree Scenes.
  double numerical_precision {1E-3}; //!< Numerical precision for triangle intersections.
  double overlap_thickness {0.0}; //!< Allowed overlap thickness for self-intersecting volumes.
  RTCDevice g_device {nullptr}; //!< Embree device object.
};

#endif
