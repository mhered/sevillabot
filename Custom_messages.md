# Custom messages

1. Create a /msg folder and declare custom message TimestampedData.msg

```
int64 timestamp
int32 data
```

2. modify CMakeLists.txt:

```python
...
# find dependencies
find_package(rosidl_default_generators REQUIRED)

# Declare ROS2 interface files
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TimestampedData.msg
  DEPENDENCIES std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
...
```

3. modify package.xml

```xml
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  ...
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

```

4. compile, source and check it is available

```bash
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 interface list | grep sevillabot
    sevillabot/msg/TimestampedData
```



Note: For some reason when the node is called from normal terminal it freezes and publishes always the same value:

```bash
$ ros2 run sevillabot line_follower_publisher.py
$ # or
$ python3 ./src/sevillabot/my_scripts/line_follower_publisher.py
```

I need to call it from vs_code terminal, and after that it runs well... why? 



