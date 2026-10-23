# One decoded serial frame, five channels in channel_ids order.
# header.stamp is the host ROS receive/decode time, NOT a hardware sample time.
# The firmware has no timestamp or sequence; source_uuid/sequence detect loss only
# after host decoding. frame_id=tcp identifies the pad geometry reference frame.
std_msgs/Header header
string source_uuid
uint64 sequence
uint8[5] channel_ids
float64[5] force_n
# Force-equivalent travel from firmware, in mm; not an independent displacement.
float64[5] travel_mm
uint16[5] ad
uint8 OK=0
uint8 SATURATED=1
uint8 OPEN=2
uint8 INVALID=3
uint8[5] status
