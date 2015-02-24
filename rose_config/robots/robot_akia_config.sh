# Location of the rose_config package
export ROSE_CONFIG="/home/rose/git/rose/shared/src/rose_config/rose_config"

# Location of the rose_tools package
export ROSE_TOOLS="/home/rose/git/rose/shared/src/rose_tools"

# Workspaces
export WORKSPACES_FILE=${ROSE_CONFIG}/workspaces/rose.ws

# Rosconsole settings
export ROSCONSOLE_CONFIG_FILE=${ROSE_TOOLS}/rosconsole_config_files/custom_rosconsole_levels.config

# Setup commands, called in ~/.bashrc
export SETUP_COMMAND_PC1="${ROSE_TOOLS}/scripts/setup_ROS.sh /opt/ros/hydro/ 10.8.0.1 http://rosepc1:11311 /home/rose/git/"
export SETUP_COMMAND_PC2="${ROSE_TOOLS}/scripts/setup_ROS.sh /opt/ros/hydro/ 10.8.0.2 http://rosepc1:11311 /home/rose/git/"

# .rosinstall location. The root of the locations that are installed by the wstool.
# If you change the next location, you have to execute first_compile again, or 
# copy the current .rosinstall file to this location.
export ROSINSTALL_ROOT="/home/rose/git/rose"

# Source external configurations

# Robot model
source ${ROSE_CONFIG}/models/model_rose21.config

# Location
source ${ROSE_CONFIG}/locations/location_engelen.config
