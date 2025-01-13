#!/bin/bash

# Usage: ./setting.sh <target_path>
# Example: ./setting.sh ~/ros2_ws/src

# Check if a parameter is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <target_path>"
  echo "Example: $0 ~/ros2_ws/src"
  exit 1
fi

# Assign the target path
TARGET_PATH=$1

# Validate that the target path exists
if [ ! -d "$TARGET_PATH" ]; then
  echo "Error: Target path '$TARGET_PATH' does not exist!"
  exit 1
fi

# Define the source directory
SOURCE_DIR=~/ros2_ws/ardupilot_simulation_example

# Validate that the source directory exists
if [ ! -d "$SOURCE_DIR" ]; then
  echo "Error: Source directory '$SOURCE_DIR' does not exist!"
  exit 1
fi

# Perform the file copies
echo "Copying directories from $SOURCE_DIR to $TARGET_PATH..."

# Copy models
if [ -d "$SOURCE_DIR/models/" ]; then
  cp -r $SOURCE_DIR/models/ $TARGET_PATH/ardupilot_gazebo/models/
  echo "Copied $SOURCE_DIR/models/ to $TARGET_PATH/ardupilot_gazebo/models/"
else
  echo "Warning: $SOURCE_DIR/models/ does not exist or is empty!"
fi

# Copy worlds
if [ -d "$SOURCE_DIR/worlds/" ]; then
  cp -r $SOURCE_DIR/worlds/ $TARGET_PATH/ardupilot_gz/ardupilot_gz_gazebo/worlds/
  echo "Copied $SOURCE_DIR/worlds/ to $TARGET_PATH/ardupilot_gz/ardupilot_gz_gazebo/worlds/"
else
  echo "Warning: $SOURCE_DIR/worlds/ does not exist or is empty!"
fi

# Copy config
if [ -d "$SOURCE_DIR/config/" ]; then
  cp -r $SOURCE_DIR/config/ $TARGET_PATH/ardupilot_gz/ardupilot_gz_bringup/config/
  echo "Copied $SOURCE_DIR/config/ to $TARGET_PATH/ardupilot_gz/ardupilot_gz_bringup/config/"
else
  echo "Warning: $SOURCE_DIR/config/ does not exist or is empty!"
fi

# Copy specific launch files
if [ -f "$SOURCE_DIR/launch/robots/iris_camera.launch.py" ]; then
  cp $SOURCE_DIR/launch/robots/iris_camera.launch.py $TARGET_PATH/ardupilot_gz/ardupilot_gz_bringup/launch/robots/
  echo "Copied iris_camera.launch.py"
else
  echo "Warning: iris_camera.launch.py does not exist!"
fi

if [ -f "$SOURCE_DIR/launch/iris_forest.launch.py" ]; then
  cp $SOURCE_DIR/launch/iris_forest.launch.py $TARGET_PATH/ardupilot_gz/ardupilot_gz_bringup/launch/
  echo "Copied iris_forest.launch.py"
else
  echo "Warning: iris_forest.launch.py does not exist!"
fi

echo "All files have been processed successfully!"
