# Workspaces
export WORKSPACES_FILE=`rospack find rose_config`/workspaces/default.workspace

# Rosconsole settings
export ROSCONSOLE_CONFIG_FILE=`rospack find rose_tools`/rosconsole_config_files/custom_rosconsole_levels.config

# Setup commands, called in ~/.bashrc
export SETUP_COMMAND_PC1="$ROSE_SCRIPTS/setup_ROS.sh /opt/ros/hydro/ 10.8.0.1 http://rosepc1:11311 /home/rose/git/"
export SETUP_COMMAND_PC2="$ROSE_SCRIPTS/setup_ROS.sh /opt/ros/hydro/ 10.8.0.2 http://rosepc1:11311 /home/rose/git/"

# Source external configurations

# Robot model
source `rospack find rose_config`/models/model_rose21.config

# Location
source `rospack find rose_config`/locations/location_rose.config