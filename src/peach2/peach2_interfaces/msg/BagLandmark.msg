# One 3D landmark on a bag (bottom / neck / tie) in the parent message frame.

uint8 SOURCE_GEOMETRY=0   # v2.0: mask width profile + 3D radius profile
uint8 SOURCE_KEYPOINT=1   # v2.1: learned keypoint

bool valid
uint8 source
geometry_msgs/Point position      # [m]
float64[9] covariance             # row-major 3x3 [m^2]
float32 confidence                # 0..1
