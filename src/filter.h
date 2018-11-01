

void intersectionFilter(void* ptr, RTCRay2 &ray) 
{

  switch(ray.rf_type) 
    {
    case 0: //if this is a typical ray_fire, check the dot_product
      if ( 0 > dot_prod(ray) )
	ray.geomID = RTC_INVALID_GEOMETRY_ID;
      break;
    case 1: //if this is a point_in_vol fire, do nothing
      break;
    }

}
