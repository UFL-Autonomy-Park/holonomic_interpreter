# holonomic_interpreter
Converts holonomic cmd_vel inputs into non-holonomic commands using a PID controller. To be used for the ground team robots at the UF Autonomy Park

## Parameters
| Parameter Name | Type | Description | Default |
| -------- | ------- | ------- | ------- |
| `heading_kp` | double | Proportional gain for PID controller | 1.0 |
| `heading_ki` | double | Integral gain for PID controller | 0.0 |
| `heading_kd` | double | Derivative Gain for PID controller | 0.1 | 
| `max_lin_vel` | double | Saturates velocity setpoint at this value [m/s] | 0.5 |
| `origin rotation` | double | Used for conversion of quaternion into yaw angle [rad] | -0.6021965548550632 |
| `control_loop_frequency` | int | Dictates update rate of non-holonomic_cmd_vel outputs [Hz] | 100 |


## Subscribers
| Topic Name | Type | Description |
| -------- | ------- | ------- |
| `holo_cmd_vel` | `Twist` | Desired holonomic velocity input |
| `odom` | `Odometry` | Global odometry from robot | 

## Publishers
| Topic Name | Type | Description |
| -------- | ------- | ------- |
| `nonholo_cmd_vel` | `Twist` | Resultant non-holonomic velocity output from PD |

## Notes
Currently just a PD controller

## TO-DO:
- [] Add integral term in controller