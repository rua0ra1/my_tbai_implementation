# tbai_mpc_perceptive package

## Example
```bash
# Start ROS and relevant nodes
roslaunch tbai_mpc_perceptive simple.launch gui:=true

# Change controllers
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'STAND'"
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'WBC'"
```




https://github.com/lnotspotl/tbai/assets/82883398/84634ba6-1ea8-49ed-9da8-50346bba2a3a



