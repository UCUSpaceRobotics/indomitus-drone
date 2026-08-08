#!/usr/bin/env bash
# install_ros2_jazzy_trixie_robust_cpp.sh
# Robust, idempotent ROS 2 Jazzy installer for Raspberry Pi OS / Debian Trixie arm64.
# Target use case: compile and run C++ ROS 2 nodes on the Pi.
#
# What this script does differently from the earlier version:
# - Uses the corrected rospian /public/ signing-key URL.
# - Forces a clean rospian APT metadata refresh to avoid stale/empty Packages files.
# - Installs a minimal C++ development stack rather than a broad desktop stack.
# - Avoids rosdep by default because it is not required for compiling your own C++ nodes.
# - Deep runtime validation: Python imports, ldd checks, ros2 CLI checks, and talker/listener communication.
# - If validation fails, it performs a one-time bulk reinstall of installed ros-jazzy-* packages and validates again.
# - Safe to rerun.

set -Eeuo pipefail

# -----------------------------
# Defaults
# -----------------------------
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
ROSPRIAN_SUITE="${ROSPRIAN_SUITE:-trixie-jazzy}"
ROSPRIAN_REPO_URL="${ROSPRIAN_REPO_URL:-https://rospian.github.io/rospian-repo}"
ROSPRIAN_KEY_URL="${ROSPRIAN_KEY_URL:-https://rospian.github.io/rospian-repo/public/rospian-archive-keyring.asc}"
ROSPRIAN_KEYRING="${ROSPRIAN_KEYRING:-/usr/share/keyrings/rospian-archive-keyring.gpg}"
ROSPRIAN_LIST="${ROSPRIAN_LIST:-/etc/apt/sources.list.d/rospian.list}"
WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ros2_ws}"
SKIP_FULL_UPGRADE="${SKIP_FULL_UPGRADE:-0}"
SKIP_WORKSPACE_BUILD="${SKIP_WORKSPACE_BUILD:-0}"
INSTALL_ROSDEP="${INSTALL_ROSDEP:-0}"        # set to 1 only if you need rosdep for external packages
RUN_COMM_TEST="${RUN_COMM_TEST:-1}"           # set to 0 to skip talker/listener communication test
FORCE_REPAIR="${FORCE_REPAIR:-0}"             # set to 1 to bulk-reinstall ros-jazzy-* even before validation

LOG_DIR="${LOG_DIR:-$HOME/ros2_jazzy_install_logs}"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/install_$(date +%Y%m%d_%H%M%S).log"

log()  { printf '\033[1;34m[INFO]\033[0m %s\n' "$*" | tee -a "$LOG_FILE"; }
warn() { printf '\033[1;33m[WARN]\033[0m %s\n' "$*" | tee -a "$LOG_FILE"; }
err()  { printf '\033[1;31m[ERROR]\033[0m %s\n' "$*" | tee -a "$LOG_FILE" >&2; }

on_error() {
  err "Installation failed near line $1. Log: $LOG_FILE"
  err "Useful diagnostics:"
  err "  cat /etc/os-release"
  err "  uname -m"
  err "  dpkg --print-architecture"
  err "  cat ${ROSPRIAN_LIST}"
  err "  sudo apt update"
  err "  apt-cache show ros-${ROS_DISTRO_NAME}-ros-base | head"
  err "  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && ros2 pkg list > /tmp/ros2_pkgs.txt"
}
trap 'on_error $LINENO' ERR

run_logged() {
  log "Running: $*"
  "$@" 2>&1 | tee -a "$LOG_FILE"
}

append_once() {
  local line="$1"
  local file="$2"
  touch "$file"
  if ! grep -Fxq "$line" "$file"; then
    printf '\n%s\n' "$line" >> "$file"
    log "Added to ${file}: ${line}"
  else
    log "Already present in ${file}: ${line}"
  fi
}

safe_source_ros() {
  # ROS setup scripts may reference unset variables. Temporarily disable nounset.
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  set -u
}

check_platform() {
  log "Checking platform..."
  local machine arch codename
  machine="$(uname -m)"
  arch="$(dpkg --print-architecture)"
  codename="$(. /etc/os-release && printf '%s' "${VERSION_CODENAME:-unknown}")"

  [[ "$machine" == "aarch64" ]] || { err "Expected uname -m=aarch64, got: $machine"; exit 1; }
  [[ "$arch" == "arm64" ]] || { err "Expected dpkg architecture arm64, got: $arch"; exit 1; }
  [[ "$codename" == "trixie" ]] || { err "Expected VERSION_CODENAME=trixie, got: $codename"; exit 1; }

  log "Platform OK: machine=${machine}, arch=${arch}, codename=${codename}"
}

install_host_prereqs() {
  log "Updating base APT metadata..."
  run_logged sudo apt update

  if [[ "$SKIP_FULL_UPGRADE" == "1" ]]; then
    warn "Skipping apt full-upgrade because SKIP_FULL_UPGRADE=1"
  else
    run_logged sudo apt full-upgrade -y
  fi

  log "Installing host prerequisites..."
  run_logged sudo apt install -y curl wget gnupg ca-certificates lsb-release build-essential cmake pkg-config
}

install_keyring() {
  log "Installing rospian signing key..."
  local tmp_key
  tmp_key="$(mktemp)"
  curl -fsSL "$ROSPRIAN_KEY_URL" -o "$tmp_key"

  if ! grep -q "BEGIN PGP PUBLIC KEY BLOCK" "$tmp_key"; then
    rm -f "$tmp_key"
    err "Downloaded key does not look like a PGP public key: $ROSPRIAN_KEY_URL"
    exit 1
  fi

  gpg --dearmor < "$tmp_key" | sudo tee "$ROSPRIAN_KEYRING" >/dev/null
  sudo chmod 0644 "$ROSPRIAN_KEYRING"
  rm -f "$tmp_key"
  log "Keyring installed at ${ROSPRIAN_KEYRING}"
}

configure_repo() {
  log "Configuring rospian APT repository..."
  local expected
  expected="deb [arch=arm64 signed-by=${ROSPRIAN_KEYRING}] ${ROSPRIAN_REPO_URL} ${ROSPRIAN_SUITE} main"

  if [[ -f "$ROSPRIAN_LIST" ]] && grep -Fxq "$expected" "$ROSPRIAN_LIST"; then
    log "Repository source already configured correctly."
  else
    printf '%s\n' "$expected" | sudo tee "$ROSPRIAN_LIST" >/dev/null
    log "Wrote repository source to ${ROSPRIAN_LIST}"
  fi
}

refresh_rospian_metadata() {
  log "Refreshing rospian APT metadata..."
  sudo rm -f /var/lib/apt/lists/rospian.github.io_*
  run_logged sudo apt clean
  run_logged sudo apt update

  local packages_file lines
  packages_file="/var/lib/apt/lists/rospian.github.io_rospian-repo_dists_${ROSPRIAN_SUITE}_main_binary-arm64_Packages"
  if [[ ! -f "$packages_file" ]]; then
    err "Expected rospian Packages file not found: $packages_file"
    exit 1
  fi
  lines="$(wc -l < "$packages_file" | tr -d ' ')"
  log "Local rospian Packages index has ${lines} lines."
  [[ "$lines" != "0" ]] || { err "rospian Packages file is 0 lines after refresh"; exit 1; }
}

verify_package_metadata() {
  log "Checking ROS package metadata..."
  apt-cache show "ros-${ROS_DISTRO_NAME}-ros-base" >/dev/null 2>&1 || {
    err "APT cannot see ros-${ROS_DISTRO_NAME}-ros-base"
    exit 1
  }
  apt-cache policy "ros-${ROS_DISTRO_NAME}-ros-base" | tee -a "$LOG_FILE"
}

install_ros_cpp_stack() {
  log "Installing minimal ROS 2 C++ development stack..."

  local pkgs=(
    "ros-${ROS_DISTRO_NAME}-ros-base"
    "ros-${ROS_DISTRO_NAME}-demo-nodes-cpp"
    "python3-colcon-ros"
  )

  if [[ "$INSTALL_ROSDEP" == "1" ]]; then
    pkgs+=("python3-rosdep2")
  fi

  run_logged sudo apt install -y "${pkgs[@]}"
}

init_rosdep_if_requested() {
  if [[ "$INSTALL_ROSDEP" != "1" ]]; then
    log "Skipping rosdep because INSTALL_ROSDEP=0. This is OK for compiling your own C++ nodes."
    return
  fi

  log "Initialising rosdep if needed..."
  if [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "rosdep already initialised."
  else
    sudo rosdep init || true
  fi
  run_logged rosdep update
}

configure_shell() {
  log "Configuring shell environment..."
  append_once "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash" "$HOME/.bashrc"
}

create_workspace() {
  if [[ "$SKIP_WORKSPACE_BUILD" == "1" ]]; then
    warn "Skipping workspace build because SKIP_WORKSPACE_BUILD=1"
    return
  fi

  log "Creating/building workspace at ${WORKSPACE_DIR}..."
  mkdir -p "${WORKSPACE_DIR}/src"
  safe_source_ros
  cd "$WORKSPACE_DIR"
  run_logged colcon build
  append_once "source ${WORKSPACE_DIR}/install/setup.bash" "$HOME/.bashrc"
}

ros_python_validation() {
  log "Validating Python-side ROS packages..."
  safe_source_ros
  python3 - <<'PY'
import ament_index_python
import rclpy
import rpyutils
import rosidl_parser
from rcl_interfaces.msg import Parameter
print("Python ROS validation OK")
PY
}

ldd_check_file() {
  local file="$1"
  local label="$2"
  if [[ ! -e "$file" ]]; then
    err "Missing ${label}: ${file}"
    return 1
  fi
  local missing
  missing="$(ldd "$file" | grep 'not found' || true)"
  if [[ -n "$missing" ]]; then
    err "Missing shared libraries for ${label}:"
    printf '%s\n' "$missing" >&2
    printf '%s\n' "$missing" >> "$LOG_FILE"
    return 1
  fi
  log "ldd check OK for ${label}"
}

ldd_runtime_validation() {
  log "Validating native shared library dependencies..."
  safe_source_ros

  ldd_check_file "/opt/ros/${ROS_DISTRO_NAME}/lib/python3.13/site-packages/rclpy/_rclpy_pybind11.cpython-313-aarch64-linux-gnu.so" "rclpy native extension"
  ldd_check_file "/opt/ros/${ROS_DISTRO_NAME}/lib/demo_nodes_cpp/talker" "demo_nodes_cpp talker"
  ldd_check_file "/opt/ros/${ROS_DISTRO_NAME}/lib/demo_nodes_cpp/listener" "demo_nodes_cpp listener"
}

ros_cli_validation() {
  log "Validating ROS CLI..."
  safe_source_ros
  command -v ros2 >/dev/null
  ros2 --help >/tmp/ros2_help.txt
  ros2 pkg list >/tmp/ros2_pkgs.txt
  local count
  count="$(wc -l < /tmp/ros2_pkgs.txt | tr -d ' ')"
  log "ros2 package count: ${count}"
  head /tmp/ros2_pkgs.txt | tee -a "$LOG_FILE"
}

comm_validation() {
  if [[ "$RUN_COMM_TEST" != "1" ]]; then
    warn "Skipping talker/listener communication test because RUN_COMM_TEST=0"
    return
  fi

  log "Running talker/listener communication test..."
  safe_source_ros

  local talker_log listener_log talker_pid
  talker_log="/tmp/ros2_talker_${$}.log"
  listener_log="/tmp/ros2_listener_${$}.log"
  rm -f "$talker_log" "$listener_log"

  ros2 run demo_nodes_cpp talker >"$talker_log" 2>&1 &
  talker_pid=$!

  # Give DDS discovery and process startup a short moment. This is not a promise to user; it is local script control flow.
  sleep 3

  set +e
  timeout 10 ros2 run demo_nodes_cpp listener >"$listener_log" 2>&1
  local listener_rc=$?
  set -e

  kill "$talker_pid" >/dev/null 2>&1 || true
  wait "$talker_pid" >/dev/null 2>&1 || true

  if grep -q "I heard" "$listener_log"; then
    log "Talker/listener communication test OK."
    head -5 "$listener_log" | tee -a "$LOG_FILE"
    return 0
  fi

  err "Talker/listener communication test failed."
  err "Talker log: $talker_log"
  err "Listener log: $listener_log"
  cat "$talker_log" >&2 || true
  cat "$listener_log" >&2 || true
  return 1
}

bulk_reinstall_ros_packages() {
  log "Bulk-reinstalling installed ros-${ROS_DISTRO_NAME}-* packages to repair missing payload files..."
  mapfile -t ros_pkgs < <(dpkg -l | awk -v prefix="ros-${ROS_DISTRO_NAME}-" '$1 == "ii" && index($2, prefix) == 1 {print $2}' | sort -u)

  if [[ "${#ros_pkgs[@]}" -eq 0 ]]; then
    err "No installed ros-${ROS_DISTRO_NAME}-* packages found to reinstall."
    exit 1
  fi

  log "Reinstalling ${#ros_pkgs[@]} ROS packages."
  run_logged sudo apt install --reinstall -y "${ros_pkgs[@]}"
}

repair_if_needed() {
  if [[ "$FORCE_REPAIR" == "1" ]]; then
    warn "FORCE_REPAIR=1: running bulk reinstall before validation."
    bulk_reinstall_ros_packages
    return
  fi

  log "Running deep validation before deciding whether repair is needed..."
  set +e
  ros_python_validation && ldd_runtime_validation && ros_cli_validation && comm_validation
  local rc=$?
  set -e

  if [[ "$rc" == "0" ]]; then
    log "Deep validation passed. No repair needed."
    return
  fi

  warn "Deep validation failed. Performing one-time bulk reinstall of installed ROS packages."
  bulk_reinstall_ros_packages
}

final_validation() {
  log "Running final validation..."
  ros_python_validation
  ldd_runtime_validation
  ros_cli_validation
  comm_validation
  log "Final validation passed."
}

main() {
  log "Starting ROS 2 Jazzy robust C++ installer. Log: $LOG_FILE"
  check_platform
  install_host_prereqs
  install_keyring
  configure_repo
  refresh_rospian_metadata
  verify_package_metadata
  install_ros_cpp_stack
  configure_shell
  init_rosdep_if_requested
  create_workspace
  repair_if_needed
  final_validation

  log "Installation complete. Open a new terminal or run: source ~/.bashrc"
}

main "$@"