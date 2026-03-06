
"use strict";

let GetCurrentState = require('./GetCurrentState.js')
let PlanTrajectory = require('./PlanTrajectory.js')
let ExecuteTrajectory = require('./ExecuteTrajectory.js')
let SetSpeedFactor = require('./SetSpeedFactor.js')
let SetRobotIO = require('./SetRobotIO.js')
let MoveToPose = require('./MoveToPose.js')
let ReadRobotIO = require('./ReadRobotIO.js')
let SetRobotEnable = require('./SetRobotEnable.js')

module.exports = {
  GetCurrentState: GetCurrentState,
  PlanTrajectory: PlanTrajectory,
  ExecuteTrajectory: ExecuteTrajectory,
  SetSpeedFactor: SetSpeedFactor,
  SetRobotIO: SetRobotIO,
  MoveToPose: MoveToPose,
  ReadRobotIO: ReadRobotIO,
  SetRobotEnable: SetRobotEnable,
};
