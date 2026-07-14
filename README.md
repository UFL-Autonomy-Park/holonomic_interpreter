# holonomic_interpreter
Converts non-holonomic cmd_vel inputs into holonomic commands using a PID controller. To be used for the ground team robots at the UF Autonomy Park

## Parameters
| Parameter Name | Type | Description | Default |
| -------- | ------- | ------- | ------- |
| `heading_kp` | double | Proportional gain for PID controller | 1.0 |
| `heading_ki` | double | Integral gain for PID controller | 0.0 |
| `heading_kd` | double | Derivative Gain for PID controller | 0.1 | 
| `max_vel` | double | Saturates velocity setpoint at this value | 0.5 |
| `min_vel` | double | Currently unused | 0.15 |
| `max_ang_acc` | double | Currently unused | 0.05 |
| `origin rotation` | double | Currently unused | -0.6021965548550632 |


## Subscribers
| Topic Name | Type | Description |
| -------- | ------- | ------- |
| `holo_cmd_vel` | `Twist` | Desired holonomic velocity input |
| `odom` | `Odometry` | Global odometry from robot | 

## Publishers
| Topic Name | Type | Description |
| -------- | ------- | ------- |
| `nonholo_cmd_vel` | `Twist` | |
| `control_error` | `Float64` |  | 
| `control_setpoint` | `Float64` |  | 
| `nonholo_cmd_vel_stamped` | `TwistWithCovarianceStamped` |  | 
| `heading` | `Float64` |  | 

## Notes
The PID controller in `src/pid.cpp` is sourced from [bradley2019's implementation](https://gist.github.com/bradley219/5373998)