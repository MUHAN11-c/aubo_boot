# Multi-view fused model of one bag in base_link. 95% uncertainties are
# already scaled (MAD * 1.4826 * 1.96).

std_msgs/Header header
string target_id
uint64 model_revision         # strictly increasing per target

BagLandmark bottom
BagLandmark neck
BagLandmark tie
geometry_msgs/Vector3 axis    # unit, bottom -> neck
float64 d95_m                 # [m]
float64 length_m              # [m] bottom -> neck along axis
float64 fruit_top_offset_m    # [m] neck -> fruit top along -axis; NaN if unknown
bool branch_direction_known
geometry_msgs/Vector3 branch_direction   # unit, twig direction above the neck (bite/shear roll)

float64 sigma_lateral95_m     # [m]
float64 sigma_axial95_m       # [m] neck position along axis
float64 theta95_deg           # [deg] axis angle
float64 swing_amplitude_m     # [m]
uint32 n_views
float32 fit_rmse_m
float32 inlier_ratio
bool converged                # sigma gates met
