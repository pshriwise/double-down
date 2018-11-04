
#include "primitives.hpp"

void intersectionFilter(void* ptr, RTCDRay &ray) 
{

  switch(ray.rf_type) 
    {
    case 0: //if this is a typical ray_fire, check the dot_product
      if ( 0 > ray.dot_prod() )
	ray.geomID = RTC_INVALID_GEOMETRY_ID;
      break;
    case 1: //if this is a point_in_vol fire, do nothing
      break;
    }

}


void DblTriBounds(void* tris_i, size_t item, RTCBounds& bounds_o) {

  const DblTri* tris = (const DblTri*) tris_i;
  const DblTri& this_tri = tris[item];  

  moab::Interface* mbi = (moab::Interface*) this_tri.moab_instance;
  moab::ErrorCode rval;

  std::vector<moab::EntityHandle> conn;
  rval = mbi->get_connectivity(&(this_tri.handle), 1, conn);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get triangle connectivity");

  if (conn.size() != 3) {
    throw std::length_error("Incorrect number of coordinates returned for a triangle entity.");
  }
  
  moab::CartVect coords[3];
  rval = mbi->get_coords(&conn[0], 1, coords[0].array());
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  rval = mbi->get_coords(&conn[1], 1, coords[1].array());
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  rval = mbi->get_coords(&conn[2], 1, coords[2].array());
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");

  double bump_val = 5e-03;
  
  bounds_o.lower_x = std::min(coords[0][0],std::min(coords[1][0],coords[2][0]));
  bounds_o.lower_y = std::min(coords[0][1],std::min(coords[1][1],coords[2][1]));
  bounds_o.lower_z = std::min(coords[0][2],std::min(coords[1][2],coords[2][2]));

  bounds_o.upper_x = std::max(coords[0][0],std::max(coords[1][0],coords[2][0]));
  bounds_o.upper_y = std::max(coords[0][1],std::max(coords[1][1],coords[2][1]));
  bounds_o.upper_z = std::max(coords[0][2],std::max(coords[1][2],coords[2][2]));
  
  bounds_o.lower_x -= bump_val; bounds_o.lower_y -= bump_val; bounds_o.lower_z -= bump_val; 
  bounds_o.upper_x += bump_val; bounds_o.upper_y += bump_val; bounds_o.upper_z += bump_val; 

  return;
}


void DblTriIntersectFunc(void* tris_i, RTCDRay& ray, size_t item) {

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

  bool hit = plucker_ray_tri_intersect(coords, ray_org, ray_dir, dist, ptr);
  
  if ( hit ) {
    ray.dtfar = dist;
    ray.tfar = dist;
    ray.u = 0.0f;
    ray.v = 0.0f;
    ray.geomID = this_tri.geomID;
    ray.primID = (unsigned int) item;

    Vec3da normal = cross((coords[1] - coords[0]),(coords[2] - coords[0]));

    if( -1 == this_tri.sense ) normal *= -1;

    ray.dNg[0] = normal[0];
    ray.dNg[1] = normal[1];
    ray.dNg[2] = normal[2];
  }

  return;
}

void DblTriOccludedFunc(void* tris_i, RTCDRay& ray, size_t item) {

  const DblTri* tris = (const DblTri*) tris_i;
  const DblTri& this_tri = tris[item];  
  RTCBounds bounds;

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
  Vec3da ray_org(ray.dorg);
  Vec3da ray_dir(ray.ddir);
  
  bool hit = plucker_ray_tri_intersect(coords, ray_org, ray_dir, dist, ptr);
  if ( hit ) {
    ray.geomID = 0;
  }
}
