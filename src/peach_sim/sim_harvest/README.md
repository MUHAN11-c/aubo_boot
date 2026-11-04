# sim_harvest

Simulation-only actions for Nav2 navigation, MoveIt planning and Gazebo fruit transfer.
The public actions are `/sim_harvest/run_harvest` (`peach_interfaces/RunHarvest`) and
`/sim_harvest/pickup` (`moveit_msgs/Pickup`). Use the project runner in `../scripts/`.
No real hardware or SetIO calls exist in this package. See `doc/architecture.md` and
`doc/io.md` for operation, failure and evidence contracts.
