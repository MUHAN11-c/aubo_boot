# Stable failure codes grouped by subsystem. Each code maps to exactly one
# handling policy in peach2_task (retry / skip / recover); never parse `reason`.
# Group: 1x perception, 2x target model / budget, 3x planning, 4x execution,
#        5x end effector, 6x safety / environment.

uint32 NONE=0

uint32 PERCEPTION_NO_TARGET=10          # skip: target not visible in any view
uint32 PERCEPTION_EXACT_TF_MISSING=11   # retry view: no TF at image stamp
uint32 PERCEPTION_LOW_QUALITY=12        # retry view: mask/depth quality gate
uint32 PERCEPTION_OUT_OF_SCOPE=13       # skip: unbagged or unsupported class

uint32 MODEL_NOT_CONVERGED=20           # skip: sigma gates not met within max views
uint32 MODEL_STALE=21                   # re-query: revision changed during cycle
uint32 MODEL_EXPIRED=22                 # re-observe: valid_until passed
uint32 BUDGET_RADIAL_NEGATIVE=23        # skip for this tool: bag too wide / too uncertain
uint32 BUDGET_AXIAL_NEGATIVE=24         # approach only, no cut
uint32 BUDGET_STRUCTURAL=25             # tool calibration makes cut impossible; fix calibration
uint32 NECK_REMEASURE_MISMATCH=26       # close-range neck differs from fused model; no cut
uint32 SWING_TOO_LARGE=27               # wait or skip
uint32 NECK_REMEASURE_PENDING=28        # cut requires a close-range neck re-measure first

uint32 PLAN_NO_IK=30                    # skip: unreachable
uint32 PLAN_COLLISION=31                # skip: blocked by hard obstacle
uint32 PLAN_FAILED=32                   # retry once
uint32 PLAN_CARTESIAN_INCOMPLETE=33     # skip: straight segment fraction < 1

uint32 EXEC_FAILED=40                   # recover
uint32 EXEC_TIMEOUT=41                  # recover
uint32 CONTACT_ABORT=42                 # retreat, skip
uint32 PREGRASP_RESIDUAL=43             # retry correction, then skip
uint32 RETREAT_FAILED=44                # recovery required
uint32 TARGET_TIMEOUT=45                # skip: per-target time budget exhausted

uint32 TOOL_NOT_OPEN=50                 # recovery required: open not confirmed
uint32 TOOL_COMMAND_FAILED=51           # recovery required
uint32 TOOL_FEEDBACK_TIMEOUT=52         # open, retreat, recovery required
uint32 CUT_NOT_CONFIRMED=53             # open, retreat, retry once
uint32 TOOL_FAULT=54                    # recovery required, tool locked
uint32 TOOL_NOT_FEASIBLE=55             # skip for this tool

uint32 SAFETY_GATE_CLOSED=60            # stop batch
uint32 ROBOT_NOT_READY=61               # stop batch
uint32 CANCELED=62                      # stop
uint32 RECOVERY_REQUIRED=63             # wait for human ack
uint32 ENVIRONMENT_UNSAFE=64            # pause batch (wind, light, rain)
uint32 DEPENDENCY_UNAVAILABLE=65        # stop batch: peer service/action server missing

uint32 code
