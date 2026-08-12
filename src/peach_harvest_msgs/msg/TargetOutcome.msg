uint8 SUCCEEDED=0
uint8 SKIPPED_QUALITY=1
uint8 SKIPPED_UNREACHABLE=2
uint8 FAILED=3
uint8 CANCELED=4

string target_id
uint8 outcome
string reason
float32 quality_score
builtin_interfaces/Duration elapsed
