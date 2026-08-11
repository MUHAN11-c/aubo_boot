# 单目标连续观测：全局目标计划中的稳定 ID + 当前帧结果/掩膜
std_msgs/Header header
string target_id
uint32 priority
bool confirmed
bool selected
string harvest_status  # PLANNED/WAITING_QUALITY/SELECTED/HARVESTED

uint8 tracking_status
uint8 OBSERVED=0
uint8 OCCLUDED=1
uint8 LOST=2
uint8 INVALID=3

float32 camera_distance_m
float32 confidence
BagGraspCandidate candidate
BagGrasp2D candidate_2d
BagFitting fitting
sensor_msgs/Image mask
string[] diagnostic_flags
