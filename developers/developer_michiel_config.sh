# Workspaces
export WORKSPACES_FILE=`rospack find rose_config`/workspaces/default.workspace

# Rosconsole settings
export ROSCONSOLE_CONFIG_FILE=`rospack find rose_tools`/rosconsole_config_files/custom_rosconsole_levels.config

# .rosinstall location. The root if the locations that are installed by the wstool.
# If you change the next location, you have to execute first_compile again.
export ROSINSTALL_ROOT="/home/osch/git/rose"

# Source external configurations

# Location
source `rospack find rose_config`/locations/location_rose.config