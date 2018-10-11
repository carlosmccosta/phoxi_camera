# Test interfaces for phoxi_camera node
## Requirements
* Installed PhoXi Control
* Installed pytest: sudo apt-get install python-pytest

Before run of tests set scanner ID or file camera ID in the **interfaces/config.py**, 
it is needed for connect to camera. Default file camera ID for testing is "InstalledExamples-PhoXi-example".


## Compile and run tests
This command is needed for compile tests in phoxi_camera package and also run all tests which have been built.
```bash
catkin_make run_tests_phoxi_camera
```

If you have compiled tests, you can only run them via rostest:
```bash
rostest -t phoxi_camera phoxi_api_interface_class.test            # unittest for PhoXiInterface class
rostest -t phoxi_camera phoxi_camera_ros_interfaces_exist.test    # this test check if all interfaces exist
rostest -t phoxi_camera phoxi_camera_ros_interfaces.test          # this test call all services of phoxi_camera node and check responses

```

Find more options of rostest:
```bash
rostest -h
```

## Output of test
After run of a test, base information from the test are written to console.
For addition information check files:

Test result:
```bash
~/.ros/test_results/phoxi_camera/rostest-tests_phoxi_camera_interfaces.xml
```

More detailed test log:
```bash
~/.ros/log/rostest-X-Y.log
```

## How it function
These tests are realized via launch files with extension .test.
In this special launch file is included tested launch file which runs
target ros node and load parameters. The special launch file also launch
testing node which consist of python unittests, this node interact with
tested node and perform tests.
