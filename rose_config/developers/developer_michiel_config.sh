# Location of the rose_config package
export ROSE_CONFIG="/home/osch/git/rose/shared/src/rose_config/rose_config"

# Location of the rose_tools package
export ROSE_TOOLS="/home/osch/git/rose/shared/src/rose_tools"

# Workspaces
export WORKSPACES_FILE=${ROSE_CONFIG}/workspaces/michiel.ws

# Rosconsole settings
export ROSCONSOLE_CONFIG_FILE=${ROSE_TOOLS}/rosconsole_config_files/custom_rosconsole_levels.config

# .rosinstall location. The root of the locations that are installed by the wstool.
# If you change the next location, you have to execute first_compile again, or 
# copy the current .rosinstall file to this location.
export ROSINSTALL_ROOT="/home/osch/git/rose"

# Source external configurations

# Location
source ${ROSE_CONFIG}/locations/location_rose.config
