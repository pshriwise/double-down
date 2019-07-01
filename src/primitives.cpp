
#include "primitives.hpp"

void intersectionFilter(void* ptr, RTCDRayHit &rayhit)
{
  switch(rayhit.ray.rf_type)
    {
    case 0: //if this is a typical ray_fire, check the dot_product
      if ( 0 > rayhit.dot_prod() )
	rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
      break;
    case 1: //if this is a point_in_vol fire, do nothing
      break;
    }
}


void DblTriBounds(const RTCBoundsFunctionArguments* args)
{
  void* tris_i = args->geometryUserPtr;
  size_t item = args->primID;
  RTCBounds& bounds_o = *args->bounds_o;

  const DblTri* tris = (const DblTri*) tris_i;
  const DblTri& this_tri = tris[item];

  moab::Interface* mbi = (moab::Interface*) this_tri.moab_instance;

  bounds_o = DblTriBounds(mbi, this_tri.handle);

  return;
}

void DblTriIntersectFunc(RTCIntersectFunctionNArguments* args) {

  void* tris_i = args->geometryUserPtr;
  size_t item = args->primID;
  RTCDRayHit* rayhit = (RTCDRayHit*)args->rayhit;
  RTCDRay& ray = rayhit->ray;
  RTCDHit& hit = rayhit->hit;

  const DblTri* tris = (const DblTri*) tris_i;
  const DblTri& this_tri = tris[item];

  moab::Interface* mbi = (moab::Interface*) this_tri.moab_instance;
  moab::ErrorCode rval;

  std::vector<moab::EntityHandle> conn;
  rval = mbi->get_connectivity(&(this_tri.handle), 1, conn);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get triangle connectivity");

  Vec3da coords[3];
  rval = mbi->get_coords(&conn[0], 1, &(coords[0][0]));
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  rval = mbi->get_coords(&conn[1], 1, &(coords[1][0]));
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  rval = mbi->get_coords(&conn[2], 1, &(coords[2][0]));
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");

  double dist;
  double nonneg_ray_len = 1e17;
  const double* ptr = &nonneg_ray_len;
  Vec3da ray_org(ray.dorg);
  Vec3da ray_dir(ray.ddir);

  bool hit_tri = plucker_ray_tri_intersect(coords, ray_org, ray_dir, dist, ptr);

  if ( hit_tri ) {
    if (ray.geomID != -1 && dist > ray.dtfar) {
      ray.geomID = -1;
      return;
    }

    ray.dtfar = dist;
    ray.tfar = dist;
    hit.u = 0.0f;
    hit.v = 0.0f;
    hit.geomID = this_tri.geomID;
    hit.primID = (unsigned int) item;

    Vec3da normal = cross((coords[1] - coords[0]),(coords[2] - coords[0]));

    if( -1 == this_tri.sense ) normal *= -1;

    hit.dNg[0] = normal[0];
    hit.dNg[1] = normal[1];
    hit.dNg[2] = normal[2];
  } else {
    hit.geomID = RTC_INVALID_GEOMETRY_ID;
  }

  return;
}

void DblTriOccludedFunc(RTCOccludedFunctionNArguments* args) {

  void* tris_i = args->geometryUserPtr;
  size_t item = args->primID;
  RTCDRay* ray = (RTCDRay*)&(args->ray);

  const DblTri* tris = (const DblTri*) tris_i;
  const DblTri& this_tri = tris[item];

  moab::Interface* mbi = (moab::Interface*) this_tri.moab_instance;
  moab::ErrorCode rval;

  std::vector<moab::EntityHandle> conn;
  rval = mbi->get_connectivity(&(this_tri.handle), 1, conn);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get triangle connectivity");

  Vec3da coords[3];
  rval = mbi->get_coords(&conn.front(), conn.size(), &(coords[0][0]));
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");

  double dist;
  double nonneg_ray_len = 1e37;
  double* ptr = &nonneg_ray_len;
  Vec3da ray_org(ray->dorg);
  Vec3da ray_dir(ray->ddir);

  bool hit_tri = plucker_ray_tri_intersect(coords, ray_org, ray_dir, dist, ptr);
  if ( hit_tri ) {
    ray->set_len(neg_inf);
  }
}

double DblTriClosestFunc(const DblTri& tri, const double loc[3]) {


  moab::Interface* mbi = (moab::Interface*) tri.moab_instance;
  moab::ErrorCode rval;

  std::vector<moab::EntityHandle> conn;
  rval = mbi->get_connectivity(&(tri.handle), 1, conn);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get triangle connectivity");

  Vec3da coords[3];
  rval = mbi->get_coords(&conn[0], 1, &(coords[0][0]));
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  rval = mbi->get_coords(&conn[1], 1, &(coords[1][0]));
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  rval = mbi->get_coords(&conn[2], 1, &(coords[2][0]));
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");

  Vec3da location(loc);

  Vec3da result;
  closest_location_on_tri(location, coords, result);

  return (result - location).length();
}
