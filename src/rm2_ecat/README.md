### Still to be realized

| Functionality                     | condition |
| --------------------------------- | --------- |
| EtherCAT bus reconnected          | √         |
| Add Hybird Joint Interface        | ×         |
| Multicycle count for DAMIAO motor | ×         |
| Separate dbus as a slave          | ×         |

### To compile this package you need some depend packages

[ecat_manager_dev](https://github.com/gdut-dynamic-x/ecat_manager_dev)

[soem_interface](https://github.com/gdut-dynamic-x/soem_interface)

[any_node](https://github.com/ANYbotics/any_node)

[message_logger](https://github.com/ANYbotics/message_logger)

### For non-root operation of this ROS package, please install the `ethercat_grant` tool as recommended.

[ethercat_grant](https://github.com/shadow-robot/ethercat_grant)(install from source)

for example:

```yaml
<node name="rm_ecat_hw" pkg="rm_ecat_ros" type="rm_ecat_hw" launch-prefix="ethercat_grant" output="screen">
  <param name="setupFile" value="path_to_your_setup_file"/>
</node>
```


