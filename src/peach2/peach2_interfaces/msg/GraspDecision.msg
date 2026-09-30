# Per-target, per-tool permission computed once by peach2_target_model and
# queried synchronously (srv GetDecision) by peach2_manipulation at CONTACT/TOOL
# stages. Grants sleeve/cut only; never commands motion.

std_msgs/Header header             # frame_id = base_link
string target_id
uint64 model_revision
string tool_id
builtin_interfaces/Time valid_until

bool approach_allowed              # pregrasp is safe to reach
bool sleeve_allowed                # radial margin > 0
bool cut_allowed                   # axial margin > 0 and fruit clearance ok
float64 radial_margin_m            # [m]
float64 axial_margin_m             # [m]

geometry_msgs/Pose pregrasp_tcp    # TCP pose at pregrasp (z = bag axis)
geometry_msgs/Point blade_target   # where the blade plane must land (neck)
float64 insert_travel_m            # [m] TCP travel so blade plane reaches blade_target

uint32 failure_code                # FailureCode.*
string reason                      # stable short phrase
