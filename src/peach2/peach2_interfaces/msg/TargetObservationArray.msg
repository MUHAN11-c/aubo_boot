# Topic: /peach/perception/observations (reliable, volatile, depth 10).

std_msgs/Header header
uint32 scene_epoch            # increments on BeginScene
bool target_set_locked
string[] locked_target_ids    # explicit locked set (empty until locked)
TargetObservation[] observations
