# OnShapeToURDF
Full disclaimer: I yoinked most of this stuff from The Construct's course on URDF :)

In most cases where you need to know how to use URDF files, you need to create a ROS version for a robot you have.
Normally, we have a full assembly of our robots in a CAD system.
These robot models are highly convoluted, and generating an accurate URDF version by hand is no easy feat.
We need a way to export automatically to URDF.
We can use Onshape-to-robot to export automatically to URDF!
```
sudo pip install onshape-to-robot
# Or you can install it in a virtual environment of python
python -m venv onshape_venv
source onshape_venv/bin/activate
# You should see now in your linux prompt the (onshape_venv)
pip install onshape-to-robot
```
## Dependencies
```
sudo add-apt-repository ppa:openscad/releases
sudo apt-get update
sudo apt-get install openscad -y 
sudo apt-get install meshlab -y 
```
The Onshape-to-robot Python module needs your API access keys to access your Onshape models.
Get the Onshape API keys here: https://dev-portal.onshape.com/keys.
Press the New Key button, then select all the checkboxes.

It will give you this time and ONLY this time the:
- access key
- private key

Save them in a safe place by copy and paste.

## Create a setup script for setting up keys:
```
cd ~/ros2_ws
touch keys.sh
chmod +x keys.sh
```
In keys.sh:
```
// Obtained at https://dev-portal.onshape.com/keys
export ONSHAPE_API=https://cad.onshape.com
export ONSHAPE_ACCESS_KEY=Your_Access_Key
export ONSHAPE_SECRET_KEY=Your_Secret_Key
```
Check that it is sourcing correctly:
```
cd ~/ros2_ws
source keys.sh
echo $ONSHAPE_API
echo $ONSHAPE_ACCESS_KEY
echo $ONSHAPE_SECRET_KEY
```
Output:
```
https://cad.onshape.com
Your_Access_Key
Your_Secret_Key
```
Create a new package for this robot named quadruped_description.
Create a folder named quadruped to house all the files needed for this export.
In this folder, quadruped, export URDF and its 3D meshes.
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake quadruped_description --dependencies urdf xacro
cd quadruped_description
mkdir quadruped
touch quadruped/config.json

mkdir launch
mkdir rviz
```
Modify the CMakelists.txt to install the folders quadruped, launch, and rviz:
```
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# INSERT THIS INTO CMakeLists.txt
install(
  DIRECTORY
    rviz
    launch
	quadruped
  DESTINATION
    share/${PROJECT_NAME}/
)
# END INSERT

ament_package()
```
Now, add the contents of the config.json file.

In config.json:
```
{
  "documentId": "33b91de06ddc91b068fcf725", # this ID is found in URL of Onshape file. ex: URL = https://cad.onshape.com/documents/33b91de06ddc91b068fcf725/w/3e9cd5a83630cb75d064813a/e/8e6a230fa3aabb1441b0aa17
  "outputFormat": "urdf",                                                                                                      # YOUR documentId ^^^
  "packageName": "quadruped_description/quadruped", # path from ROS workspace (ie: robot_ws) to folder where generated urdf is located
  "robotName": "quadruped_robot",
  "assemblyName": "quadruped_v1" # name of assembly in onshape
}
```
Now you are ready to export.
Execute the following commands:
```
cd ~/ros2_ws/src/quadruped_description
# Execute the export
onshape-to-robot quadruped
```
Among generated files, you should have a file named robot.urdf.
Insert the following at the top of robot.urdf (Make this line 1, robot_name should be line 2):
```
<?xml version="1.0"?>
```

## ROS launch
Fill in the code for the ROS2 launch files.

In launch/quadruped.launch.py: Assign package_description, robot_desc_path variables to respective pkg / robot names

In launch/start_rviz.launch.py: Assign package_description, rviz_config_dir variables to respective pkg / robot names 

- TODO what to do if there's no .rviz file

## Running Generated Robot
In shell 1:
```
cd ~/ros2_ws/
source install/setup.bash
colcon build --packages-select quadruped_description
source install/setup.bash
ros2 launch quadruped_description quadruped.launch.py
```
In shell 2:
```
cd ~/ros2_ws/
source install/setup.bash
ros2 launch quadruped_description start_rviz.launch.py
```
In shell 3:
```
cd ~/ros2_ws/
source install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
In RVIZ, you should see the robot, and be able to move the joints around using joint_state_publisher_gui.

For more info or if something doesn't work, check out the docs: https://onshape-to-robot.readthedocs.io/en/latest/
