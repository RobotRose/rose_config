# Location of the rose_config package
export ROSE_CONFIG="/home/mathijs/git/rose/shared/src/rose_config/rose_config"

# Location of the rose_tools package
export ROSE_TOOLS="/home/mathijs/git/rose/shared/src/rose_tools"

# Workspaces
export WORKSPACES_FILE=${ROSE_CONFIG}/workspaces/mathijs.ws

# Rosconsole settings
export ROSCONSOLE_CONFIG_FILE=${ROSE_TOOLS}/rosconsole_config_files/custom_rosconsole_levels.config

# .rosinstall location. The root of the locations that are installed by the wstool.
# If you change the next location, you have to remote the old .rosinstall at the current location. Secondly,
# one should run update_rosinstall.sh from the scripts folder
export ROSINSTALL_ROOT="/home/mathijs/git/rose"

# Rosinstall file I want to use
export ROSINSTALL_CONFIG=${ROSE_CONFIG}/rosinstall/rosinstall_mathijs

# Source external configurations

# Location
source ${ROSE_CONFIG}/locations/location_rose.config
