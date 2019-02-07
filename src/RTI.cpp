
// MOAB
#include "MBTagConventions.hpp"
#include "moab/GeomTopoTool.hpp"

// Local
#include "RTI.hpp"
#include "AABB.h"

struct Node
{
  virtual float sah() = 0;
};



  struct InnerNode : public Node
  {
    AABB bounds[2];
    Node* children[2];

    InnerNode() {
      bounds[0] = bounds[1] = empty;
      children[0] = children[1] = nullptr;
    }


    float sah() {
      return 1.0f + (area(bounds[0])*children[0]->sah() + area(bounds[1])*children[1]->sah())/area(merge(bounds[0],bounds[1]));
    }

    static void* create (RTCThreadLocalAllocator alloc, size_t numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      void* ptr = rtcThreadLocalAlloc(alloc,sizeof(InnerNode),16);
      return (void*) new (ptr) InnerNode;
    }

    static void  setChildren (void* nodePtr, void** childPtr, size_t numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      for (size_t i=0; i<2; i++)
        ((InnerNode*)nodePtr)->children[i] = (Node*) childPtr[i];
    }

    static void  setBounds (void* nodePtr, const RTCBounds** bounds, size_t numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      for (size_t i=0; i<2; i++)
        ((InnerNode*)nodePtr)->bounds[i] = *(const AABB*) bounds[i];
    }
  };

  struct LeafNode : public Node
  {
    unsigned id;
    AABB bounds;

    LeafNode (unsigned id, const AABB& bounds)
      : id(id), bounds(bounds) {}

    float sah() {
      return 1.0f;
    }

    static void* create (RTCThreadLocalAllocator alloc, const RTCBuildPrimitive* prims, size_t numPrims, void* userPtr)
    {
      std::cout << "Creating leaf node with " << numPrims << " primitives.\n";
      assert(numPrims == 1);
      void* ptr = rtcThreadLocalAlloc(alloc,sizeof(LeafNode),16);
      return (void*) new (ptr) LeafNode(prims->primID,*(AABB*)prims);
    }
  };

template<typename T>
struct StackItemT {

  inline static void xchg(StackItemT& a, StackItemT& b) {
    std::swap(a,b);
  }

  inline friend void sort(StackItemT& s1, StackItemT& s2) {
    if (s1.dist < s2.dist) { xchg(s2, s1); }
  }

  inline friend void sort(StackItemT& s1, StackItemT& s2, StackItemT& s3) {
    if (s2.dist < s1.dist) xchg(s2,s1);
    if (s3.dist < s2.dist) xchg(s3,s2);
    if (s2.dist < s1.dist) xchg(s2,s1);
  }

  inline friend void sort(StackItemT& s1, StackItemT& s2, StackItemT& s3, StackItemT& s4) {
    if (s2.dist < s1.dist) xchg(s2,s1);
    if (s4.dist < s3.dist) xchg(s4,s3);
    if (s3.dist < s1.dist) xchg(s3,s1);
    if (s4.dist < s2.dist) xchg(s4,s2);
    if (s3.dist < s2.dist) xchg(s3,s2);
  }

public:
  T ptr;
  unsigned dist;

};

typedef StackItemT<Node*> StackItem;

moab::ErrorCode RayTracingInterface::init(std::string filename) {
  moab::ErrorCode rval;

  if ("" == filename && NULL == MBI) {
    return moab::MB_FAILURE;
  } else if (!MBI) {
    MBI = new moab::Core();
    rval = MBI->load_file(filename.c_str());
    MB_CHK_SET_ERR(rval, "Failed to load the specified MOAB file");
  }

  rtcInit();

  moab::Range vols;
  rval = get_vols(vols);
  MB_CHK_SET_ERR(rval, "Failed to get MOAB volumes.");

  std::cout << "Found " << vols.size() << " volumes." << std::endl;
  // set the EH offset
  sceneOffset = *vols.begin();

  moab::GeomTopoTool* GTT = new moab::GeomTopoTool(MBI);
  // create an Embree geometry instance for each surface
  for (moab::Range::iterator i = vols.begin(); i != vols.end(); i++) {
    moab::EntityHandle vol = *i;

    // add to scenes vec
    RTCScene scene = rtcNewScene(RTC_SCENE_ROBUST , RTC_INTERSECT1);
    scenes.push_back(scene);
    scene_map[vol] = scene;

    // get volume surfaces
    moab::Range surfs;
    rval = MBI->get_child_meshsets(vol, surfs);
    MB_CHK_SET_ERR(rval, "Failed to get the volume sufaces.");

    for (moab::Range::iterator j = surfs.begin(); j != surfs.end(); j++) {
      moab::EntityHandle this_surf = *j;
      // find the surface sense
      int sense;
      rval = GTT->get_sense(this_surf, vol, sense);
      MB_CHK_SET_ERR(rval, "Failed to get sense for surface");

      // get all triangles on this surface
      moab::Range tris;
      rval = MBI->get_entities_by_type(this_surf, moab::MBTRI, tris);
      MB_CHK_SET_ERR(rval, "Failed to get surface triangles.");

      int num_tris = tris.size();

      // create a new geometry for the volume's scene
      unsigned int emsurf = rtcNewUserGeometry(scene, num_tris);

      DblTri* emtris = (DblTri*) malloc(num_tris*sizeof(DblTri));

      rtcSetUserData(scene, emsurf, emtris);

      buffer_storage.store(vol, num_tris, emtris);

      for (int k = 0; k < num_tris; k++) {
        emtris[k].moab_instance = MBI;
        emtris[k].handle = tris[k];
        emtris[k].surf = this_surf;
        emtris[k].geomID = emsurf;
        emtris[k].sense = sense;
      } // end tris loop

      rtcSetBoundsFunction(scene, emsurf, (RTCBoundsFunc)&DblTriBounds);
      rtcSetIntersectFunction(scene, emsurf, (RTCIntersectFunc)&MBDblTriIntersectFunc);
      rtcSetOccludedFunction(scene, emsurf, (RTCOccludedFunc)&DblTriOccludedFunc);

    } // end surface loop

    rtcCommit(scene);

    buildBVH(vol);

  } // end volume loop

  delete GTT;

  return moab::MB_SUCCESS;
}

void RayTracingInterface::buildBVH(moab::EntityHandle vol) {

  std::cout << "Building tree for volume with handle: " << vol << "\n";
  RTCDevice device = rtcNewDevice();

  RTCBVH bvh = rtcNewBVH(device);

  /* settings for BVH build */
  const size_t extraSpace = 1000000;
  RTCBuildSettings settings;
  settings.size = sizeof(settings);
  settings.quality = RTC_BUILD_QUALITY_HIGH;
  settings.maxBranchingFactor = 2;
  settings.maxDepth = 1024;
  settings.sahBlockSize = 1;
  settings.minLeafSize = 1;
  settings.maxLeafSize = 1;
  settings.travCost = 1.0f;
  settings.intCost = 1.0f;
  settings.extraSpace = extraSpace;

  std::pair<int, DblTri*> buffer = buffer_storage.retrieve_buffer(vol);

  std::vector<RTCBuildPrimitive> prims;
  prims.resize(buffer.first);

  for(int i = 0; i < buffer.first; i++) {
    DblTri dtri = buffer.second[i];

    RTCBounds bounds = DblTriBounds((moab::Interface*)dtri.moab_instance,
                                    dtri.handle);
    RTCBuildPrimitive prim;
    prim.lower_x = bounds.lower_x;
    prim.lower_y = bounds.lower_y;
    prim.lower_z = bounds.lower_z;
    prim.geomID = 0;
    prim.upper_x = bounds.upper_x;
    prim.upper_y = bounds.upper_y;
    prim.upper_z = bounds.upper_z;
    prim.primID = i;
    prims[i] = prim;
  }

  Node* root = (Node*) rtcBuildBVH(bvh,
                                   settings,
                                   prims.data(),
                                   prims.size(),
                                   InnerNode::create,
                                   InnerNode::setChildren,
                                   InnerNode::setBounds,
                                   LeafNode::create,
                                   NULL,
                                   NULL,
                                   NULL);
  root_map[vol] = root;

}

void RayTracingInterface::closest(moab::EntityHandle vol, const double loc[3],
                                  double &result, moab::EntityHandle* surface) {


  std::pair<int, DblTri*> buffer = buffer_storage.retrieve_buffer(vol);

  size_t stackSize = 1024;
  StackItem stack[stackSize];
  StackItem* stackPtr = stack+1;
  StackItem* stackEnd = stack + stackSize;

  const float floc[3] = {loc[0], loc[1], loc[2]};

  stack[0].ptr = root_map[vol];
  stack[0].dist = neg_inf;

  Vec3da closest;
  double current_result = inf;
  while (true) pop:
    {
      // stack is empty, we're done
      if(stackPtr == stack) break;

      // move to next node
      stackPtr--;
      Node* cur = stackPtr->ptr;

      // we've found a location closer than this node, move on
      if(stackPtr->dist > current_result) { continue; }

      while (true)
        {
          InnerNode* inode = dynamic_cast<InnerNode*>(cur);
          if (!inode) { break; }
          // check nodes
          float d1 = inode->bounds[0].nearest(floc);
          float d2 = inode->bounds[1].nearest(floc);

          stackPtr->ptr = inode->children[0];
          stackPtr->dist = d1;

          stackPtr++;

          stackPtr->ptr = inode->children[1];
          stackPtr->dist = d2;

          sort(*(stackPtr),*(stackPtr-1));

          cur = stackPtr->ptr;
        }

      LeafNode* leaf = dynamic_cast<LeafNode*>(cur);
      // at leaf
      DblTri this_prim = buffer.second[leaf->id];

      double tmp = DblTriClosestFunc(this_prim, loc);
      std::cout << "TMP : " << tmp << std::endl;
      double current_result = std::min(tmp, current_result);
    }

  result = current_result;
  return;

}

void RayTracingInterface::shutdown() {
  for(auto s : scenes) { rtcDeleteScene(s); }

  if(MBI) { delete MBI; }

  rtcExit();
}

void RayTracingInterface::fire(moab::EntityHandle vol, RTCDRay &ray) {

  rtcIntersect(scenes[vol-sceneOffset], *((RTCRay*)&ray));

}

void RayTracingInterface::dag_point_in_volume(const moab::EntityHandle volume,
                                              const double xyz[3],
                                              int& result,
                                              const double *uvw,
                                              moab::GeomQueryTool::RayHistory *history,
                                              double overlap_tol,
                                              moab::EntityHandle imp_comp) {
  const double huge_val = std::numeric_limits<double>::max();
  double dist_limit = huge_val;

  RTCScene scene = scene_map[volume];

  double dir[3];
  if (uvw) {
    dir[0] = uvw[0];
    dir[1] = uvw[1];
    dir[2] = uvw[2];
  } else {
    dir[0] = 0.5;
    dir[1] = 0.5;
    dir[2] = 0.5;
  }

  MBRay mbray;
  mbray.set_org(xyz);
  mbray.set_dir(dir);

  mbray.tnear = 0.0;
  mbray.set_len(dist_limit);
  mbray.geomID = RTC_INVALID_GEOMETRY_ID;
  mbray.primID = RTC_INVALID_GEOMETRY_ID;
  mbray.rf_type = RayFireType::PIV;
  mbray.orientation = 1;
  mbray.mask = -1;
  if (history) { mbray.rh = history; }

  // fire ray
  rtcIntersect(scene, *((RTCRay*)&mbray));

  Vec3da ray_dir(dir);
  Vec3da tri_norm(mbray.dNg[0], mbray.dNg[1], mbray.dNg[2]);

  if (mbray.geomID != RTC_INVALID_GEOMETRY_ID) {
    result = dot(ray_dir, tri_norm) > 0.0 ? 1 : 0;
  }
  else {
    result = 0;
  }

  if (overlap_tol != 0.0) {
    MBRayAccumulate aray;
    aray.set_org(xyz);
    aray.set_dir(dir);
    aray.instID = volume;
    aray.set_len(dist_limit);
    aray.rf_type = RayFireType::ACCUM;
    aray.sum = 0;
    aray.num_hit = 0;

    rtcIntersect(scene, *((RTCRay*)&aray));
    if (aray.num_hit == 0) { result = 0; return; }
    int hits = aray.num_hit;

    aray.geomID = -1;
    aray.primID = -1;
    aray.set_len(dist_limit);
    aray.tnear = 0.0;
    aray.set_dir(aray.ddir*-1);
    rtcIntersect(scene, *((RTCRay*)&aray));
    if (aray.num_hit == hits) { result = 0; return; }
    // inside/outside depends on the sum
    if      (0 < aray.sum)                                          result = 0; // pt is outside (for all vols)
    else if (0 > aray.sum)                                          result = 1; // pt is inside  (for all vols)
    else if (imp_comp && imp_comp == volume)                        result = 1;
    else                                                            result = 0;
  }
}


void RayTracingInterface::dag_ray_fire(const moab::EntityHandle volume,
                                       const double point[3],
                                       const double dir[3],
                                       moab::EntityHandle& next_surf,
                                       double& next_surf_dist,
                                       moab::GeomQueryTool::RayHistory* history,
                                       double user_dist_limit,
                                       int ray_orientation) {

  const double huge_val = std::numeric_limits<double>::max();
  double dist_limit = huge_val;
  if (user_dist_limit > 0) { dist_limit = user_dist_limit; }

  next_surf = 0; next_surf_dist = huge_val;

  RTCScene scene = scene_map[volume];

  MBRay mbray;
  mbray.set_org(point);
  mbray.set_dir(dir);
  mbray.tnear = 0.0;
  mbray.set_len(dist_limit);
  mbray.geomID = RTC_INVALID_GEOMETRY_ID;
  mbray.primID = RTC_INVALID_GEOMETRY_ID;
  mbray.rf_type = RayFireType::RF;
  mbray.orientation = ray_orientation;
  mbray.mask = -1;
  if (history) { mbray.rh = history; }

  // fire ray
  rtcIntersect(scene, *((RTCRay*)&mbray));
  // check behind the ray origin for intersections
  double neg_ray_len = 1e-03;

  MBRay neg_ray;
  neg_ray.set_org(point);
  neg_ray.set_dir(-mbray.ddir);
  neg_ray.tnear = 0.0;
  neg_ray.geomID = -1;
  neg_ray.primID = -1;
  neg_ray.rf_type = RayFireType::RF;
  neg_ray.orientation = -ray_orientation;
  if (history) { neg_ray.rh = history; }
  neg_ray.set_len(neg_ray_len);

  rtcIntersect(scene, *((RTCRay*)&neg_ray));

  bool use_neg_intersection = false;
  // If an RTI is found at negative distance, perform a PMT to see if the
  // particle is inside an overlap.
  if(neg_ray.geomID != RTC_INVALID_GEOMETRY_ID) {
    moab::ErrorCode rval;
    // get the next volume
    std::vector<moab::EntityHandle> vols;
    moab::EntityHandle nx_vol;
    rval = MBI->get_parent_meshsets( neg_ray.surf_handle, vols );
    MB_CHK_SET_ERR_CONT(rval, "Failed to get the parent meshsets");
    if (2 != vols.size()) {
      MB_CHK_SET_ERR_CONT(moab::MB_FAILURE, "Invaid number of parent volumes found");
    }
    if(vols.front() == volume) {
      nx_vol = vols.back();
    } else {
      nx_vol = vols.front();
    }
    int result = 0;
    dag_point_in_volume( nx_vol, point, result, dir, history );
    MB_CHK_SET_ERR_CONT(rval, "Point in volume query failed");
    if (1==result) use_neg_intersection = true;
  }

  if(use_neg_intersection) {
    next_surf_dist = 0;
    next_surf = neg_ray.surf_handle;
  }
  else if ( mbray.geomID != RTC_INVALID_GEOMETRY_ID) {
    next_surf_dist = mbray.dtfar;
    next_surf = mbray.surf_handle;
  }
  else {
    next_surf_dist = 1E37;
    next_surf = 0;
  }

  if(history) {
    if(use_neg_intersection) {
      history->add_entity(neg_ray.prim_handle);
    }
    else {
      history->add_entity(mbray.prim_handle);
    }
  }

}

moab::ErrorCode RayTracingInterface::get_vols(moab::Range& vols) {
  moab::ErrorCode rval;

  // retrieve vols
  moab::Tag geom_tag;
  rval = MBI->tag_get_handle("GEOM_DIMENSION", geom_tag);
  MB_CHK_SET_ERR(rval, "Failed to get the geometry dimension tag");

  int val = 3;
  const void *dum = &val;
  rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_tag, &dum, 1, vols);
  MB_CHK_SET_ERR(rval, "Failed to get all surface sets in the model");

  return moab::MB_SUCCESS;
}
