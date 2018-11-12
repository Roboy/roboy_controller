rostopic pub /Roboy/MoveEndEffector/head/goal roboy_communication_control/MoveEndEffectorActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  endEffector: 'head'
  type: 2
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
  target_frame: ''
  sendToRealHardware: true
  q_target: [0,0,0,0.6]
  timeout: 5
  tolerance: 0.01"
