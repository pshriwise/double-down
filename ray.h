
// This might go away with the alorithm update
enum rf_type { RF, PIV};

struct RTCRay2 : RTCRay { int rf_type; };

// TO-DO: there should be a few more double elements here (barycentric coords)
struct RTCDRay: RTCRay2 { double dorg[3]; double ddir[3]; double dtfar;};
