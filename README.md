# biometra_module

Github repository for driver and client calling functions for the Biometra TRobot II

## biometra_driver

- The biometra_driver is a python interface which allows the user to call functions from the Biometra library found at `biometra_module/biometra_driver/biometra_driver/dotnet/BiometraLibraryNet.xml`

- Through the biometra_driver, the user can connect to their thermocycler and send commands such as open/close_lid, run_program, and get_status

### biometra.py

- contains the parent function `run_program` which is called by the client when running a thermocycler program. 

- `run_program` takes a single argument "prog" which indicates the id number of the desired protocol already stored on the connected biometra

- `run_program` will call several functions in `functions.py` in order to close the biometra lid, run the program given, open the lid, and ensure that the block is relatively cool (50 deg)

- TODO: will pass any outputted errors to errors.py for decoding

### functions.py

- Locates all of the available thermocyclers plugged into the computer
    - Calls `find_device` function to select which thermocycler to use
    - TODO: currently hardcoded to select the first device available on list of devices
- Runs `login_user` function to login the admin. All protocols available on the biometra are stored under a specific username
- Contains all python functions used to access the biometra library
    - Some usefull functions that the user may find helpful:
        - `get_status`: returns "READY", "BUSY", or "ERROR" depending on the current state of the block
        - `list_programs`: prints all of the available programs found on the connected biometra, their names, id numbers, and date created.
        - `stop_program`: ends the current program if one is currently running on the thermocycler
        - `open_lid`: opens the thermocycler lid
        - `close_lid`: closes the thermocycler lid
        - `run_pcr_program`: (args: prog) inputs program id integer and runs the given program 
        - `get_run_status`: prints whether there is an active program on the connected thermocycler
        - `get_lid_state`: prints whether the lid is open or closed
        - `get_time_left`: prints the time remaining on the current running program
        - `check_temp_lid`: prints the temperature of the thermocycler lid
        - `check_temp_all`: prints a list of the temperatures of the left, middle, and right blocks of the thermocycler

### errors.py

- TODO: will decode any error messages generated after running a program

## biometra_client

A ROS2 wrapper that accepts service calls from the wei_client in order to call certain functions on the thermocycler

Start biometra node:

`ros2 launch biometra_client biometra_client.launch.py`

Accepted commands:
- `open_lid`
- `close_lid`
- `run_program`
- `get_status`

## Running
The following are step by step instructions for activating the biometra ROS2 node and running a protocol

- `ssh rpl@parker`
- navigate to `~/wei_ws`
- `ros2 launch biometra_client biometra_client.launch.py`
- wait to ensure biometra node is broadcasting "READY" state
- switch back to local computern (sched)
- navigate to `~/home/workspace/`
- `source /opt/ros/humble/setup.bash`
- `source ~/wei_ws/install/setup.bash`
- `./` + path to your wei client

## Troubleshooting
#### `find_device` function 'index out of range': 
- The biometra cannot be found connected to any COM port, check the wired connection and try running again

#### biometra is saying the lid is open when it is not and vice versa
- power cycle the biometra

