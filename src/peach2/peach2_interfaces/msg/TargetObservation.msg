# Single-frame observation of one tracked target. No accept/reject verdict:
# fusion and budget decide.
# header.frame_id = base_link (z up); header.stamp = image acquisition stamp.

uint8 CATEGORY_BAG=0
uint8 CATEGORY_NOBAG=1       # display only, never an execution candidate

std_msgs/Header header
string target_id
uint8 category
bool confirmed                     # track confirmed (N consecutive associations)

BagLandmark bottom
BagLandmark neck
BagLandmark tie                    # neck tip / tie point; often occluded
geometry_msgs/Vector3 axis         # unit, bottom -> neck; zero if unknown
float64 diameter95_m               # [m] 95th percentile section diameter
float64 camera_distance_m          # [m]
geometry_msgs/Pose camera_pose     # camera optical frame in base_link at the image stamp

float32 mask_quality               # 0..1 (area ratio, completeness)
float32 depth_coverage             # 0..1 valid+confident depth inside mask
bool edge_touch                    # mask touches image border
bool swing_known                   # false = amplitude/period not estimated yet
float64 swing_amplitude_m          # [m] from stationary window
float64 swing_period_s             # [s]
sensor_msgs/RegionOfInterest roi
string[] flags                     # diagnostic tags, informational only
