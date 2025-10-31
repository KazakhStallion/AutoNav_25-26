
# Python file to run the test.

# Step 1 create an empty log file containing the test launch time and name
# 001.log
# Have this file contained in /logs stored on the Jetson
# path.parent.mkdir(datetime)
# with path.open("w", encoding="utf-8") as log_file:

# Step 2 publish to the topic "/data/toggle_collect" the boolean TRUE
# This sould tell the data_collection node to start publishing data to the
# "/data/dump" Topic.

# Step 2.5 start the test which involves t001 related actions.
# Setting GPS waypoints, running the command to go to the waypoints in order, returning to start, turning around
# This depends on which ROS2 Nodes and Launch files the t001_GPS_Calibration.launch.py sets up

# Step 3 grab all the data from the "/data/dump" Topic LIVE during test 

# Step 4 reset the "/data/toggle_collect" topic by sending the boolean FALSE
# This should stop the data_publisher.cpp Node

# Step 5 format data to go out to log file in CSV format

# Important, have an ESTOP callback to terminate the code in the 2.5 step when the ESTOP node is activated.