# CNOSolarROS2
## Prerequisites 
- ### A Linux-based operating system (e.g., Ubuntu)
- ### Administrative privileges (sudo access) on your system
## Install ros2 humble Ubuntu(debian)
### To install ros 2 follow this [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), if you don't have too much storage in the machine is possible to install just the ROS-base version. 
## Add sourcing to your shell startup script [[reference]](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
### If you don’t want to have to source the setup file every time you open a new shell, then you can add the command to your shell startup script:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
### to have access to colcon variables using tap you can also add this to the shell script
```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```
## Clone the git repository
### Create a workspace folder (e.g, Cnosolar_ws) and clone the repository using:
```bash
git clone git@github.com:AngelDiaz03/CNOSolarROS2.git
```
## Run the project
### Inside the workspace folder, you need to build the project for that use: 
```bash
colcon build --symlink-install 
```
### Note: The "--symlink-install" flag is to make easier the working process whe you use it, there's no need to build agin every time you change a file.

### Then source the insatll folder using: 
```bash
source install/setup.bash
```
### Is also recommendable to add the source of the install folder to the shell. 
```bash
echo "source {$PATH}/install/setup.bash" >> ~/.bashrc
```
### where $PATH is the global path to your workspace. 
### Finally to run the project just use the launch file: 
```bash
ros2 launch meteorology_data meteorology.launch.xml
```
## Main possible changes
- ### Meteorology data file: 
    ### To change the data .csv file go to src/meteorology_data/resource and add the fille that you need. 
    ### Then go to src/meteorology_data/meteorology_data/meteorology_node and change the "self.data_file" variable to access the new file. 
- ### Configuration .json file 
    ### To change the config .json file go to src/cnsolar/resource and add the fille that you need. 
    ### Then go to src/meteorology_data/meteorology_data/solar_park_node and change the "self.json_str" path to match the name of the new file. 
## Future work 
- ### Change meteorology node to get real data in real time. 
- ### Cahnge bridge node to publish the data into a database. 
