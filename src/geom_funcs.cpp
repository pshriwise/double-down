
double dot_prod( RTCRay &ray )
{

  float result = ray.dir[0]*ray.Ng[0];
  result += ray.dir[1]*ray.Ng[1];
  result += ray.dir[2]*ray.Ng[2]; 
  
  return result;

}
