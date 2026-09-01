#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${SCRIPT_DIR}"
DEBUG_LOG_FILE="${WORKSPACE_ROOT}/trae-debug-log-runros-shell-loop.ndjson"
ROS_LOG_ROOT="${WORKSPACE_ROOT}/log/ros2"
DEFAULT_LAUNCH_PACKAGE="odin_ros_driver"
DEFAULT_LAUNCH_FILE="odin1_ros2.launch.py"

# Function: print_info
# Input:
#   - $1: message text.
# Output:
#   - Prints an informational message to stdout.
# Description:
#   - Standardizes normal runtime messages for this script.
# References:
#   - None.
print_info() {
    echo "[INFO] $1"
}

# Function: print_warn
# Input:
#   - $1: warning text.
# Output:
#   - Prints a warning message to stdout.
# Description:
#   - Standardizes warning messages for this script.
# References:
#   - None.
print_warn() {
    echo "[WARN] $1"
}

# Function: print_error
# Input:
#   - $1: error text.
# Output:
#   - Prints an error message to stderr.
# Description:
#   - Standardizes error messages for this script.
# References:
#   - None.
print_error() {
    echo "[ERROR] $1" >&2
}

# Function: write_debug_log
# Input:
#   - $1: event name.
#   - $2: event detail text.
# Output:
#   - Appends one NDJSON debug record to the local debug log file.
# Description:
#   - Records runtime evidence for script invocation mode, environment loading,
#     and interactive shell handoff without changing business behavior.
# References:
#   - Uses variables `DEBUG_LOG_FILE`, `WORKSPACE_ROOT`, and `ROS_DISTRO`
#     defined in this file.
#   - Called by `load_ros2_environment()`, `load_workspace_environment()`,
#     `start_interactive_shell()`, and `main()` in this file.
write_debug_log() {
    local event_name="$1"
    local event_detail="$2"
    local invoke_mode="executed"

    if is_script_sourced; then
        invoke_mode="sourced"
    fi

    printf '{"ts":"%s","event":"%s","detail":"%s","pid":"%s","ppid":"%s","mode":"%s","shell":"%s","workspace":"%s","ros_distro":"%s"}\n' \
        "$(date '+%Y-%m-%dT%H:%M:%S%z')" \
        "${event_name}" \
        "${event_detail}" \
        "$$" \
        "$PPID" \
        "${invoke_mode}" \
        "${SHELL:-/bin/bash}" \
        "${WORKSPACE_ROOT}" \
        "${ROS_DISTRO:-unset}" >> "${DEBUG_LOG_FILE}"
}

# Function: is_script_sourced
# Input:
#   - None.
# Output:
#   - Returns 0 when the script is sourced.
#   - Returns 1 when the script is executed directly.
# Description:
#   - Detects whether the current script is loaded into the caller shell or
#     started as a standalone process.
# References:
#   - Uses bash built-in variables `${BASH_SOURCE[0]}` and `${0}`.
is_script_sourced() {
    [[ "${BASH_SOURCE[0]}" != "${0}" ]]
}

# Function: find_ros2_setup
# Input:
#   - None.
# Output:
#   - Prints the absolute path of the detected ROS 2 `setup.bash`.
#   - Returns 1 if no ROS 2 installation is found.
# Description:
#   - Prefers the current `ROS_DISTRO` when available, otherwise probes common
#     ROS 2 distributions from newer to older.
# References:
#   - Uses environment variable `ROS_DISTRO`.
#   - Searches under `/opt/ros/<distro>/setup.bash`.
find_ros2_setup() {
    local ros2_setup=""
    local distros=("jazzy" "iron" "humble" "galactic" "foxy" "rolling")
    local distro=""

    if [ -n "${ROS_DISTRO}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        echo "/opt/ros/${ROS_DISTRO}/setup.bash"
        return 0
    fi

    for distro in "${distros[@]}"; do
        if [ -f "/opt/ros/${distro}/setup.bash" ]; then
            ros2_setup="/opt/ros/${distro}/setup.bash"
            echo "${ros2_setup}"
            return 0
        fi
    done

    return 1
}

# Function: load_ros2_environment
# Input:
#   - None.
# Output:
#   - Returns 0 when the ROS 2 environment is sourced successfully.
#   - Returns 1 when no valid ROS 2 environment is found.
# Description:
#   - Locates and loads the base ROS 2 environment required by the workspace.
# References:
#   - Calls `find_ros2_setup()` in this file.
#   - Sources `/opt/ros/<distro>/setup.bash`.
load_ros2_environment() {
    local ros2_setup=""

    ros2_setup="$(find_ros2_setup)" || {
        write_debug_log "load_ros2_environment_failed" "no_ros2_setup_found"
        print_error "No ROS 2 installation was found under /opt/ros."
        return 1
    }

    # shellcheck disable=SC1090
    source "${ros2_setup}"
    write_debug_log "load_ros2_environment" "${ros2_setup}"
    print_info "Loaded ROS 2 environment: ${ros2_setup}"
    return 0
}

# Function: load_workspace_environment
# Input:
#   - None.
# Output:
#   - Returns 0 after attempting to load the workspace environment.
# Description:
#   - Loads the current workspace overlay when `install/setup.bash` exists.
#     When the workspace has not been built yet, it keeps only the base ROS 2
#     environment and prints the recommended next step.
# References:
#   - Uses variable `WORKSPACE_ROOT` defined in this file.
#   - Sources `${WORKSPACE_ROOT}/install/setup.bash`.
load_workspace_environment() {
    local workspace_setup="${WORKSPACE_ROOT}/install/setup.bash"

    cd "${WORKSPACE_ROOT}" || return 1

    if [ -f "${workspace_setup}" ]; then
        # shellcheck disable=SC1090
        source "${workspace_setup}"
        write_debug_log "load_workspace_environment" "${workspace_setup}"
        print_info "Loaded workspace environment: ${workspace_setup}"
    else
        write_debug_log "load_workspace_environment_missing" "${workspace_setup}"
        print_warn "Workspace overlay not found: ${workspace_setup}"
        print_warn "Run 'colcon build' first if you need package overlays."
    fi

    export ROS_WORKSPACE="${WORKSPACE_ROOT}"
    print_info "Workspace root: ${WORKSPACE_ROOT}"
    return 0
}

# Function: ensure_ros_log_directory
# Input:
#   - None.
# Output:
#   - Returns 0 when the ROS 2 log directory is ready for use.
#   - Returns 1 when the log directory cannot be created.
# Description:
#   - Ensures ROS 2 launch logs are written into the workspace-local log
#     directory instead of relying on the user's home directory.
# References:
#   - Uses variables `ROS_LOG_ROOT` and `WORKSPACE_ROOT` defined in this file.
#   - Called by `main()` in this file.
ensure_ros_log_directory() {
    mkdir -p "${ROS_LOG_ROOT}" || {
        write_debug_log "ensure_ros_log_directory_failed" "${ROS_LOG_ROOT}"
        print_error "Failed to create ROS log directory: ${ROS_LOG_ROOT}"
        return 1
    }

    export ROS_LOG_DIR="${ROS_LOG_ROOT}"
    write_debug_log "ensure_ros_log_directory" "${ROS_LOG_DIR}"
    print_info "ROS log directory: ${ROS_LOG_DIR}"
    return 0
}

# Function: run_command_with_environment
# Input:
#   - $@: command and arguments to execute.
# Output:
#   - Replaces the current process with the provided command.
# Description:
#   - Executes a user-specified command after the ROS 2 and workspace
#     environment have been loaded successfully.
# References:
#   - Called by `main()` in this file.
run_command_with_environment() {
    write_debug_log "run_command_with_environment" "$*"
    print_info "Running command with ROS 2 environment loaded: $*"
    exec "$@"
}

# Function: launch_default_ros2_stack
# Input:
#   - None.
# Output:
#   - Replaces the current process with the default ROS 2 launch command.
# Description:
#   - Starts the default `odin_ros_driver` ROS 2 launch file after the
#     workspace environment has been loaded successfully.
# References:
#   - Uses variables `DEFAULT_LAUNCH_PACKAGE` and `DEFAULT_LAUNCH_FILE`
#     defined in this file.
#   - Calls `run_command_with_environment()` in this file.
launch_default_ros2_stack() {
    write_debug_log "launch_default_ros2_stack" "${DEFAULT_LAUNCH_PACKAGE} ${DEFAULT_LAUNCH_FILE}"
    print_info "Starting default ROS 2 launch: ${DEFAULT_LAUNCH_PACKAGE} ${DEFAULT_LAUNCH_FILE}"
    run_command_with_environment ros2 launch "${DEFAULT_LAUNCH_PACKAGE}" "${DEFAULT_LAUNCH_FILE}"
}

# Function: start_interactive_shell
# Input:
#   - None.
# Output:
#   - Replaces the current process with a clean interactive bash shell.
# Description:
#   - Keeps the loaded ROS 2 and workspace environment in a new interactive
#     shell when the script is executed directly, while avoiding user shell
#     rc files that may recursively modify configuration.
# References:
#   - Uses `/bin/bash --noprofile --norc -i`.
start_interactive_shell() {
    write_debug_log "start_interactive_shell" "exec_clean_interactive_bash"
    print_info "Starting a clean interactive bash shell with ROS 2 environment loaded."
    exec /bin/bash --noprofile --norc -i
}

# Function: print_usage
# Input:
#   - None.
# Output:
#   - Prints the script usage text to stdout.
# Description:
#   - Documents the default one-click launch behavior, the custom command mode,
#     and the explicit shell mode for this script.
# References:
#   - Uses variables `DEFAULT_LAUNCH_PACKAGE` and `DEFAULT_LAUNCH_FILE`
#     defined in this file.
print_usage() {
    cat <<EOF
Usage:
  source ${WORKSPACE_ROOT}/runros.sh
  ${WORKSPACE_ROOT}/runros.sh
  ${WORKSPACE_ROOT}/runros.sh --shell
  ${WORKSPACE_ROOT}/runros.sh <command> [args...]

Behavior:
  - source runros.sh
      Load ROS 2 and workspace environment into the current shell.
  - runros.sh
      Launch: ros2 launch ${DEFAULT_LAUNCH_PACKAGE} ${DEFAULT_LAUNCH_FILE}
  - runros.sh --shell
      Open a clean interactive bash shell with the environment loaded.
  - runros.sh <command> [args...]
      Run the provided command with the environment loaded.
EOF
}

# Function: main
# Input:
#   - None.
# Output:
#   - Returns 0 on success.
#   - Returns 1 when required environment loading fails.
# Description:
#   - Coordinates ROS 2 base environment loading, workspace overlay loading,
#     and chooses behavior for sourced vs executed usage.
# References:
#   - Calls `is_script_sourced()` in this file.
#   - Calls `load_ros2_environment()` in this file.
#   - Calls `load_workspace_environment()` in this file.
#   - Calls `ensure_ros_log_directory()` in this file.
#   - Calls `launch_default_ros2_stack()` in this file.
#   - Calls `run_command_with_environment()` in this file.
#   - Calls `start_interactive_shell()` in this file.
#   - Calls `print_usage()` in this file.
main() {
    write_debug_log "main_enter" "argv:$*"

    if ! is_script_sourced; then
        case "$1" in
            -h|--help)
                print_usage
                return 0
                ;;
        esac
    fi

    load_ros2_environment || return 1
    load_workspace_environment || return 1
    ensure_ros_log_directory || return 1

    if is_script_sourced; then
        if [ "$#" -gt 0 ]; then
            write_debug_log "main_warn" "arguments_ignored_when_sourced"
            print_warn "Arguments are ignored when the script is sourced."
        fi
        write_debug_log "main_exit" "current_shell_ready"
        print_info "Environment is ready in the current shell."
        return 0
    fi

    if [ "$#" -eq 0 ]; then
        write_debug_log "main_handoff" "default_launch_requested"
        launch_default_ros2_stack
    fi

    if [ "$1" = "--shell" ]; then
        write_debug_log "main_handoff" "interactive_shell_requested"
        start_interactive_shell
    fi

    if [ "$#" -gt 0 ]; then
        write_debug_log "main_handoff" "command_execution_requested"
        run_command_with_environment "$@"
    fi
}

main "$@"
