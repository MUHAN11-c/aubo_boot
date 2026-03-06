
"use strict";

let Digital = require('./Digital.js');
let Analog = require('./Analog.js');
let GoalPoint = require('./GoalPoint.js');
let JointPos = require('./JointPos.js');
let JointTrajectoryFeedback = require('./JointTrajectoryFeedback.js');
let IOState = require('./IOState.js');
let TraPoint = require('./TraPoint.js');

module.exports = {
  Digital: Digital,
  Analog: Analog,
  GoalPoint: GoalPoint,
  JointPos: JointPos,
  JointTrajectoryFeedback: JointTrajectoryFeedback,
  IOState: IOState,
  TraPoint: TraPoint,
};
