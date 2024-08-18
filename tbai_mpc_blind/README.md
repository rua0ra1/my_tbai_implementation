# tbai_mpc_blind package

## Example
```bash
# Start ROS and relevant nodes
roslaunch tbai_mpc_blind simple.launch gui:=true

# Change controllers
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'STAND'"
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'WBC'"
```



https://github.com/lnotspotl/tbai/assets/82883398/991b6615-1b5f-4b01-9a83-40ca4bcacff6

