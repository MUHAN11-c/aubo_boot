# Per-target result, split by stage (Bac 2014 style).

uint8 OUTCOME_SUCCEEDED=0
uint8 OUTCOME_SKIPPED=1
uint8 OUTCOME_FAILED=2
uint8 OUTCOME_CANCELED=3

uint8 REACHED_NONE=0
uint8 REACHED_PREGRASP=1
uint8 REACHED_INSERTED=2
uint8 REACHED_CUT_CONFIRMED=3
uint8 REACHED_RETREATED=4
uint8 REACHED_RELEASED=5

string target_id
string tool_id
uint8 outcome
uint8 reached
uint32 failure_code          # FailureCode.*
string reason
bool recovery_required
bool plan_only               # execution disabled: planned but nothing was moved or actuated
float64 cycle_time_s
string[] stage_names
float64[] stage_times_s
float64 radial_margin_m
float64 axial_margin_m
