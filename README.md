[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# node_test

The package `node_test` extends `rostest` to add more testing functionalities at node level.

**Author & Maintainer**: Anthony Remazeilles, anthony.remazeilles@tecnalia.com

**Affiliation** : Tecnalia Research and Innovation, Spain

**License**: This project is under the Apache 2.0 License.
See [LICENSE.md](LICENSE.md) for more details.

## Getting started

### test_service

We just highlight the example usage

```shell
# in shell 1
roscore
# in shell 2
rosrun rospy_tutorials add_two_ints_server
# in shell 3
roslaunch test_service launch_test.launch
```

In the [launch file](node_test/launch/launch_test.launch) is indicated the test to perform:

```xml
<launch>
    <node name="service_test" pkg="node_test" type="test_service" output="screen" >
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

### test_filter

`test_filter` enables testing a filter-like node, that is supposed to publish a message after having processed a received message.

An example is provided in [test_filer.launch](node_test/launch/test_filter.launch):

```xml
<launch>
    <node name="dummy_node" pkg="node_test" type="dummy_filter" output="screen">
        <param name="wait" value="0.2" />
    </node>
    <node name="filter_test" pkg="node_test" type="test_filter"
        output="screen" args="--text">
        <param name="topic_in" value="/filter_in" />
        <param name="topic_out" value="/filter_out" />
        <rosparam param="msg_in"> {'data': 2.0}</rosparam>
        <rosparam param="msg_out">{'data': 4.0}</rosparam>
        <rosparam param="timeout">1.0</rosparam>
    </node>
</launch>
```

`dummy_node` is a simple filter, which double a received value.
Parameter `wait` enables emulating the processing time before publishing the result.

The parameters of `test_filter` node are:

* `topic_in`: topic to which the filter expects the message
* `topic_out`: topic to which the filter will publish the filter result
* `msg_in`; input message to be transmitted to the filter, written as a python dictionary
* `msg_out`: output message that should be published by the filter
* `timeout`: maximum time expected for the generation of the filter output
