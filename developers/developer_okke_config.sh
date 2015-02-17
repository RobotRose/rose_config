# Workspaces
export WORKSPACES_FILE=`rospack find rose_config`/workspaces/default.workspace

# Rosconsole settings
export ROSCONSOLE_CONFIG_FILE=`rospack find rose_tools`/rosconsole_config_files/custom_rosconsole_levels.config

# Source external configurations

# Location
source `rospack find rose_config`/locations/location_rose.config