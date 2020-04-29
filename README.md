[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# rostest service

The package `test_service` extends `rostest` to check a ROS service.

**Author & Maintainer**: Anthony Remazeilles, anthony.remazeilles@tecnalia.com

**Affiliation** : Tecnalia Research and Innovation, Spain

**License**: This project is under the Apache 2.0 License.
See [LICENSE.md](LICENSE.md) for more details.

## Getting started

### Usage

We just highlight the example usage

```shell
# in shell 1
roscore
# in shell 2
rosrun rospy_tutorials add_two_ints_server
# in shell 3
roslaunch test_service launch_test.launch
```

In the [launch file](test_service/launch/launch_test.launch) is indicated the test to perform:

```xml
<launch>
    <node name="service_test" pkg="test_service" type="test_service" output="screen" >
        <param name="service_name" value="/add_two_ints" />
        <rosparam param="service_input">{'a': 0, 'b': 5.0}</rosparam>
        <rosparam param="service_output">{'sum': 5}</rosparam>
    </node>
</launch>
```

where:

* `service_name`: is the service to test
* `service_input`: input parameters, defined as a dictionary
* `service_output`: expected output, defined as a dictionary

The test will connect to the service indicated, call it with the provided parameters, and compare the output received with the one defined.
The test succeeds if all went well.