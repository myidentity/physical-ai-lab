#!/bin/bash

# Isaac Sim 5.1.0 Auto Start Script with Package Management
# Based on Official NVIDIA Documentation
# https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html

# Color codes for better UI
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Official NVIDIA Isaac Sim paths
ISAAC_HOME="${HOME}/docker/isaac-sim"
CUSTOM_IMAGE="isaac-sim-custom:latest"
OFFICIAL_IMAGE="nvcr.io/nvidia/isaac-sim:5.1.0"
CONTAINER_NAME="isaac-sim"

# Status flags (set by check_prerequisites)
IMAGE_AVAILABLE=false
ISAAC_LAB_AVAILABLE=false

# Check which image to use
if docker images | grep -q "isaac-sim-custom.*latest"; then
    ISAAC_IMAGE="${CUSTOM_IMAGE}"
    USING_CUSTOM=true
else
    ISAAC_IMAGE="${OFFICIAL_IMAGE}"
    USING_CUSTOM=false
fi

# Function to print colored messages
print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_important() {
    echo -e "${CYAN}★${NC} $1"
}

# Function to create helper script for container
create_container_helper() {
    # Remove if it exists as directory (bug fix)
    if [ -d "${ISAAC_HOME}/container_helper.sh" ]; then
        print_warning "Removing incorrectly created directory..."
        sudo rm -rf "${ISAAC_HOME}/container_helper.sh"
    fi

    sudo tee "${ISAAC_HOME}/container_helper.sh" > /dev/null << 'HELPER_EOF'
#!/bin/bash
# Isaac Sim Container Helper Script

clear
echo ""
echo "  ═════════════════════════════════════════════════════════"
echo "       Isaac Sim Container - Python Package Guide"
echo "  ═════════════════════════════════════════════════════════"
echo ""
echo "  📦 INSTALLING PYTHON PACKAGES:"
echo "     ./python.sh -m pip install package_name"
echo ""
echo "  ⚠️  IMPORTANT: Packages installed are TEMPORARY!"
echo "     When you exit this container, they will be LOST."
echo ""
echo "  💾 TO SAVE YOUR PACKAGES:"
echo "     1. Keep this container running"
echo "     2. Open NEW terminal on HOST"
echo "     3. Run: docker commit isaac-sim isaac-sim-custom:latest"
echo "     4. Next time you run auto_start.sh, it will use your custom image"
echo ""
echo "  📝 EXAMPLE:"
echo "     ./python.sh -m pip install numpy pandas matplotlib"
echo ""
echo "  ═════════════════════════════════════════════════════════"
echo ""
HELPER_EOF
    sudo chmod +x "${ISAAC_HOME}/container_helper.sh"
}

# Function to create persistent directories
setup_directories() {
    print_info "Setting up Isaac Sim directories (Official NVIDIA structure)..."
    echo ""

    mkdir -p ~/docker/isaac-sim/cache/main/ov
    mkdir -p ~/docker/isaac-sim/cache/main/warp
    mkdir -p ~/docker/isaac-sim/cache/computecache
    mkdir -p ~/docker/isaac-sim/config
    mkdir -p ~/docker/isaac-sim/data/documents
    mkdir -p ~/docker/isaac-sim/data/Kit
    mkdir -p ~/docker/isaac-sim/logs
    mkdir -p ~/docker/isaac-sim/pkg

    if [ $? -eq 0 ]; then
        print_success "Directories created"
        echo ""

        # Create helper script BEFORE changing ownership
        create_container_helper

        print_info "Setting ownership to UID 1234:1234 (required by container)..."
        sudo chown -R 1234:1234 ~/docker/isaac-sim

        if [ $? -eq 0 ]; then
            print_success "Ownership configured correctly"
        else
            print_error "Failed to set ownership. Run manually:"
            echo "  sudo chown -R 1234:1234 ~/docker/isaac-sim"
            read -p "Press Enter to continue..."
            return 1
        fi

        echo ""
        echo "Directory Structure:"
        echo "  ~/docker/isaac-sim/"
        echo "    ├── cache/main/"
        echo "    ├── config/"
        echo "    ├── data/"
        echo "    ├── logs/"
        echo "    ├── pkg/"
        echo "    └── container_helper.sh (Python package guide)"
        echo ""
    else
        print_error "Failed to create directories"
        return 1
    fi
}

# Function to check prerequisites
check_prerequisites() {
    print_info "Checking prerequisites..."

    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed"
        exit 1
    fi
    print_success "Docker is installed"

    if ! docker ps &> /dev/null; then
        print_error "Docker daemon is not running or permission denied"
        exit 1
    fi
    print_success "Docker daemon is running"

    if ! docker run --rm --gpus all ubuntu nvidia-smi &> /dev/null; then
        print_error "Cannot access GPU in Docker"
        exit 1
    fi
    print_success "GPU access verified"

    # Check for custom image (don't exit if missing - user can pull from menu)
    if [ "$USING_CUSTOM" = true ]; then
        print_success "Using custom image: ${CUSTOM_IMAGE}"
        IMAGE_AVAILABLE=true
    else
        if docker images | grep -q "isaac-sim.*5.1.0"; then
            print_success "Using official image: ${OFFICIAL_IMAGE}"
            IMAGE_AVAILABLE=true
        else
            print_warning "Isaac Sim 5.1.0 image not found"
            print_info "Use menu option 'p' to pull the image"
            IMAGE_AVAILABLE=false
        fi
    fi

    # Check for Isaac Lab
    if [ -d "${HOME}/docker/isaac-lab/IsaacLab" ]; then
        ISAAC_LAB_AVAILABLE=true
    else
        ISAAC_LAB_AVAILABLE=false
    fi

    echo ""
}

# Function to show system info
show_system_info() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "           Isaac Sim 5.1.0 - System Information"
    echo "═══════════════════════════════════════════════════════"
    echo ""

    print_info "GPU Information:"
    docker run --rm --gpus all ubuntu nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null
    echo ""

    print_info "Docker Images:"
    echo "  Official: $(docker images | grep 'nvcr.io/nvidia/isaac-sim' | grep '5.1.0' | awk '{print $3, $4, $5}')"
    if docker images | grep -q "isaac-sim-custom.*latest"; then
        echo "  Custom:   $(docker images | grep 'isaac-sim-custom' | grep 'latest' | awk '{print $3, $4, $5}')"
    else
        echo "  Custom:   Not created yet"
    fi
    echo ""

    print_info "Currently Using:"
    echo "  ${ISAAC_IMAGE}"
    echo ""

    print_info "Storage:"
    if [ -d "${ISAAC_HOME}" ]; then
        du -sh "${ISAAC_HOME}" 2>/dev/null | awk '{print "  " $1 " - " $2}'
    else
        echo "  Not yet initialized"
    fi
    echo ""

    read -p "Press Enter to continue..."
}

# Function to save container changes
save_container_changes() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "    Isaac Sim: Save Container Changes (Commit Image)"
    echo "═══════════════════════════════════════════════════════"
    echo ""
    print_important "NOTE: This is for Isaac Sim containers ONLY"
    print_info "Isaac Lab uses volumes - changes persist automatically"
    echo ""

    # Check if container is running
    if ! docker ps | grep -q "${CONTAINER_NAME}"; then
        print_error "No running Isaac Sim container found!"
        echo ""
        echo "To save Isaac Sim changes:"
        echo "  1. Start Isaac Sim (Option 1 or 2)"
        echo "  2. Install packages: ./python.sh -m pip install package_name"
        echo "  3. Keep container running"
        echo "  4. Run this option (7) from menu or another terminal"
        echo ""
        read -p "Press Enter to continue..."
        return
    fi

    print_warning "This will save Isaac Sim container state to a new image"
    echo ""
    print_info "Current running container: ${CONTAINER_NAME} (Isaac Sim)"
    print_info "Will save as: ${CUSTOM_IMAGE}"
    echo ""

    read -p "Continue? (y/N): " confirm

    if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
        print_info "Committing container changes..."
        docker commit ${CONTAINER_NAME} ${CUSTOM_IMAGE}

        if [ $? -eq 0 ]; then
            print_success "Image saved successfully!"
            echo ""
            print_important "Next time you run this script, it will automatically use:"
            echo "  ${CUSTOM_IMAGE}"
            echo ""
            print_info "Your installed Python packages are now permanent!"
        else
            print_error "Failed to commit image"
        fi
    else
        print_info "Operation cancelled"
    fi

    echo ""
    read -p "Press Enter to continue..."
}

# Function to manage Python packages
manage_packages() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "           Python Package Management Guide"
    echo "═══════════════════════════════════════════════════════"
    echo ""

    print_info "Current Image: ${ISAAC_IMAGE}"
    echo ""

    echo "📦 HOW TO INSTALL PACKAGES (Isaac Sim Only):"
    echo ""
    echo "1. Start Isaac Sim (Option 1 or 2)"
    echo "2. Inside container, run:"
    echo "   ./python.sh -m pip install numpy pandas scikit-learn"
    echo ""
    echo "3. ⚠️  IMPORTANT: Keep container running!"
    echo ""
    echo "4. Open NEW terminal and run:"
    echo "   ./auto_start.sh → Option 7 (Save Container Changes)"
    echo ""
    echo "5. Exit container"
    echo "   Next run will use your custom image with packages"
    echo ""
    echo "═══════════════════════════════════════════════════════"
    echo ""
    echo "💡 ALTERNATIVE: Use requirements.txt (Advanced)"
    echo ""

    if [ -f "${ISAAC_HOME}/requirements.txt" ]; then
        print_success "Found: ~/docker/isaac-sim/requirements.txt"
        echo ""
        echo "Contents:"
        cat "${ISAAC_HOME}/requirements.txt"
        echo ""
    else
        echo "Create: ~/docker/isaac-sim/requirements.txt"
        echo "Example contents:"
        echo "  numpy==1.24.3"
        echo "  pandas==2.0.3"
        echo "  matplotlib==3.7.2"
        echo ""
    fi

    echo "═══════════════════════════════════════════════════════"
    echo ""
    read -p "Press Enter to continue..."
}

# ============ SETUP FUNCTIONS (First Time) ============

# Function to login to NGC
ngc_login() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "              NGC Login (NVIDIA GPU Cloud)"
    echo "═══════════════════════════════════════════════════════"
    echo ""
    print_info "NGC login is required to pull NVIDIA container images"
    echo ""
    echo "📋 STEPS TO GET YOUR API KEY:"
    echo ""
    echo "  1. Go to: https://ngc.nvidia.com"
    echo "  2. Create a free account (or sign in)"
    echo "  3. Click your username (top right) → Setup"
    echo "  4. Click 'Generate API Key'"
    echo "  5. Copy the key (starts with 'nvapi-...')"
    echo ""
    echo "═══════════════════════════════════════════════════════"
    echo ""
    read -p "Press Enter when you have your API key ready..."
    echo ""
    print_important "When prompted:"
    echo "  Username: \$oauthtoken  (type this literally)"
    echo "  Password: <paste your API key>"
    echo ""

    docker login nvcr.io

    if [ $? -eq 0 ]; then
        echo ""
        print_success "NGC login successful!"
    else
        echo ""
        print_error "NGC login failed. Please try again."
    fi
    echo ""
    read -p "Press Enter to continue..."
}

# Function to pull Isaac Sim image
pull_isaac_sim_image() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "         Pull Isaac Sim 5.1.0 Docker Image"
    echo "═══════════════════════════════════════════════════════"
    echo ""

    # Check if already exists
    if docker images | grep -q "isaac-sim.*5.1.0"; then
        print_success "Isaac Sim 5.1.0 image already exists!"
        echo ""
        docker images | grep isaac-sim
        echo ""
        read -p "Press Enter to continue..."
        return
    fi

    print_warning "This will download ~15GB. Make sure you have:"
    echo "  • Logged into NGC (Option s)"
    echo "  • Stable internet connection"
    echo "  • ~20GB free disk space"
    echo ""

    read -p "Continue with download? (y/N): " confirm
    if [ "$confirm" != "y" ] && [ "$confirm" != "Y" ]; then
        print_info "Download cancelled"
        read -p "Press Enter to continue..."
        return
    fi

    echo ""
    print_info "Pulling Isaac Sim 5.1.0..."
    print_info "This will take 15-30 minutes depending on your connection"
    echo ""

    docker pull nvcr.io/nvidia/isaac-sim:5.1.0

    if [ $? -eq 0 ]; then
        echo ""
        print_success "Isaac Sim 5.1.0 downloaded successfully!"
        IMAGE_AVAILABLE=true

        # Update ISAAC_IMAGE variable
        ISAAC_IMAGE="${OFFICIAL_IMAGE}"
    else
        echo ""
        print_error "Download failed. Check your NGC login and try again."
    fi
    echo ""
    read -p "Press Enter to continue..."
}

# Function to clone Isaac Lab
clone_isaac_lab() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "            Clone Isaac Lab Repository"
    echo "═══════════════════════════════════════════════════════"
    echo ""

    # Check if already exists
    if [ -d "${HOME}/docker/isaac-lab/IsaacLab" ]; then
        print_success "Isaac Lab already exists at:"
        echo "  ~/docker/isaac-lab/IsaacLab"
        echo ""
        ISAAC_LAB_AVAILABLE=true
        read -p "Press Enter to continue..."
        return
    fi

    print_info "This will clone Isaac Lab from GitHub"
    echo "  Repository: https://github.com/isaac-sim/IsaacLab.git"
    echo "  Location:   ~/docker/isaac-lab/IsaacLab"
    echo ""

    read -p "Continue? (y/N): " confirm
    if [ "$confirm" != "y" ] && [ "$confirm" != "Y" ]; then
        print_info "Clone cancelled"
        read -p "Press Enter to continue..."
        return
    fi

    echo ""
    print_info "Cloning Isaac Lab..."

    mkdir -p ~/docker/isaac-lab
    cd ~/docker/isaac-lab

    git clone https://github.com/isaac-sim/IsaacLab.git

    if [ $? -eq 0 ]; then
        echo ""
        print_success "Isaac Lab cloned successfully!"
        ISAAC_LAB_AVAILABLE=true
    else
        echo ""
        print_error "Clone failed. Check your internet connection."
    fi

    cd - > /dev/null
    echo ""
    read -p "Press Enter to continue..."
}

# ============ END SETUP FUNCTIONS ============

# Function to run with GUI
run_with_gui() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "     Isaac Sim 5.1.0 - GUI Mode (Official NVIDIA)"
    echo "═══════════════════════════════════════════════════════"
    echo ""

    # Check if image is available
    if [ "$IMAGE_AVAILABLE" != true ]; then
        print_error "Isaac Sim image not found!"
        echo ""
        print_info "Please use option 'p' to pull the image first"
        print_info "You may also need to login to NGC first (option 's')"
        echo ""
        read -p "Press Enter to continue..."
        return
    fi

    # Check if directories exist AND if container_helper.sh is properly created as a file
    if [ ! -d "${ISAAC_HOME}" ] || [ ! -f "${ISAAC_HOME}/container_helper.sh" ]; then
        setup_directories
        if [ $? -ne 0 ]; then
            return
        fi
    fi

    # Reminder about packages
    if [ "$USING_CUSTOM" = false ]; then
        print_warning "Using official image - Python packages will be temporary"
        print_info "Install packages? See Option 9 for instructions"
        echo ""
    else
        print_success "Using custom image - Your packages are saved!"
        echo ""
    fi

    print_info "Configuring X11 access (xhost +local:)..."
    xhost +local: > /dev/null 2>&1

    print_info "Starting Isaac Sim container..."
    print_warning "First run: shader compilation takes 5-10 minutes"
    echo ""
    print_important "Inside container, run: ./runapp.sh"
    print_important "Or run: ./container_helper.sh (for Python package guide)"
    echo ""

    docker run --name ${CONTAINER_NAME} --entrypoint bash -it --gpus all \
        -e "ACCEPT_EULA=Y" --rm --network=host \
        -e "PRIVACY_CONSENT=Y" \
        -v $HOME/.Xauthority:/isaac-sim/.Xauthority \
        -e DISPLAY \
        -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
        -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
        -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
        -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
        -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
        -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
        -v ~/docker/isaac-sim/container_helper.sh:/isaac-sim/container_helper.sh:ro \
        -u 1234:1234 \
        ${ISAAC_IMAGE}

    # After container exits
    echo ""
    if [ "$USING_CUSTOM" = false ]; then
        print_warning "Container exited - any installed packages are LOST"
        print_info "To save packages: Option 8 (while container is running)"
        echo ""
        read -p "Press Enter to continue..."
    fi
}

# Function to run headless
run_headless() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "   Isaac Sim 5.1.0 - Headless Mode (Official NVIDIA)"
    echo "═══════════════════════════════════════════════════════"
    echo ""

    # Check if image is available
    if [ "$IMAGE_AVAILABLE" != true ]; then
        print_error "Isaac Sim image not found!"
        echo ""
        print_info "Please use option 'p' to pull the image first"
        print_info "You may also need to login to NGC first (option 's')"
        echo ""
        read -p "Press Enter to continue..."
        return
    fi

    # Check if directories exist AND if container_helper.sh is properly created as a file
    if [ ! -d "${ISAAC_HOME}" ] || [ ! -f "${ISAAC_HOME}/container_helper.sh" ]; then
        setup_directories
        if [ $? -ne 0 ]; then
            return
        fi
    fi

    if [ "$USING_CUSTOM" = false ]; then
        print_warning "Using official image - Python packages will be temporary"
        echo ""
    fi

    print_info "Starting Isaac Sim container..."
    echo ""

    docker run --name ${CONTAINER_NAME} --entrypoint bash -it --gpus all \
        -e "ACCEPT_EULA=Y" --rm --network=host \
        -e "PRIVACY_CONSENT=Y" \
        -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
        -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
        -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
        -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
        -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
        -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
        -v ~/docker/isaac-sim/container_helper.sh:/isaac-sim/container_helper.sh:ro \
        -u 1234:1234 \
        ${ISAAC_IMAGE}
}

# Function to run Isaac Lab (Official NVIDIA Method)
run_isaac_lab_official() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "   Isaac Lab - Official NVIDIA Docker Container"
    echo "   (Includes Isaac Sim 5.1.0 + Isaac Lab)"
    echo "═══════════════════════════════════════════════════════"
    echo ""

    # Check if Isaac Lab directory exists
    if [ ! -d "${HOME}/docker/isaac-lab/IsaacLab" ]; then
        print_error "Isaac Lab not found at ~/docker/isaac-lab/IsaacLab"
        echo ""
        print_info "Please use option 'c' from the main menu to clone Isaac Lab"
        echo ""
        read -p "Press Enter to continue..."
        return
    fi

    print_success "Isaac Lab repository found"
    echo ""
    print_info "Starting official Isaac Lab container..."
    print_info "This will:"
    echo "  • Build Isaac Lab container (first time: ~10-15 min)"
    echo "  • Download Isaac Sim 5.1.0 base image if needed"
    echo "  • Start container in detached mode"
    echo "  • Enter interactive shell"
    echo ""
    read -p "Press Enter to continue..."

    # Navigate to Isaac Lab directory
    cd "${HOME}/docker/isaac-lab/IsaacLab"

    # Create log file
    LOG_FILE="${HOME}/docker/isaac-lab/isaac_lab_container.log"
    print_info "Logging output to: ${LOG_FILE}"
    echo ""

    # Run official Isaac Lab container commands
    print_info "Running: python3 docker/container.py start"
    echo ""
    python3 docker/container.py start 2>&1 | tee "${LOG_FILE}"

    if [ ${PIPESTATUS[0]} -eq 0 ]; then
        echo ""
        print_success "Container started successfully"
        echo ""
        print_info "Entering container..."
        echo ""
        python3 docker/container.py enter 2>&1 | tee -a "${LOG_FILE}"
    else
        echo ""
        print_error "Failed to start container"
        echo ""
        print_warning "Check logs at: ${LOG_FILE}"
        echo ""
        print_info "Last 20 lines of log:"
        echo "─────────────────────────────────────────────────────"
        tail -20 "${LOG_FILE}"
        echo "─────────────────────────────────────────────────────"
        echo ""
        read -p "Press Enter to continue..."
    fi

    # Return to original directory
    cd - > /dev/null
}

# Compatibility check function
run_compatibility_check() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "      Isaac Sim 5.1.0 - Compatibility Check"
    echo "═══════════════════════════════════════════════════════"
    echo ""

    # Check if image is available
    if [ "$IMAGE_AVAILABLE" != true ]; then
        print_error "Isaac Sim image not found!"
        echo ""
        print_info "Please use option 'p' to pull the image first"
        print_info "You may also need to login to NGC first (option 's')"
        echo ""
        read -p "Press Enter to continue..."
        return
    fi

    print_info "Configuring X11 access..."
    xhost +local: > /dev/null 2>&1

    print_info "Running system compatibility check..."
    echo ""

    docker run --entrypoint bash -it --gpus all --rm --network=host \
        -e "PRIVACY_CONSENT=Y" \
        -v $HOME/.Xauthority:/isaac-sim/.Xauthority \
        -e DISPLAY \
        nvcr.io/nvidia/isaac-sim:5.1.0 ./isaac-sim.compatibility_check.sh

    echo ""
    print_info "Look for: 'System checking result: PASSED'"
    echo ""
    read -p "Press Enter to continue..."
}

# Display driver check
check_display_driver() {
    if [ -f /usr/share/X11/xorg.conf.d/20-dummy.conf ]; then
        echo "dummy"
    else
        echo "nvidia"
    fi
}

# Check Isaac Sim container status
check_isaac_sim_status() {
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "running"
    else
        echo "stopped"
    fi
}

# Check Isaac Lab container status
check_isaac_lab_status() {
    if docker ps --format '{{.Names}}' | grep -q '^isaac-lab-base$'; then
        echo "running"
    elif docker ps -a --format '{{.Names}}' | grep -q '^isaac-lab-base$'; then
        echo "stopped"
    else
        echo "not_created"
    fi
}

# Toggle display driver
toggle_display_driver() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "          Display Driver Configuration"
    echo "═══════════════════════════════════════════════════════"
    echo ""

    CURRENT_DRIVER=$(check_display_driver)

    echo "════════════════════ INFORMATION ══════════════════════"
    echo ""
    print_info "Current Driver: ${CURRENT_DRIVER^^}"
    echo ""

    if [ "$CURRENT_DRIVER" = "dummy" ]; then
        echo "📺 DUMMY DRIVER (Headless Mode)"
        echo "   ✓ Works without physical monitor"
        echo "   ✓ Perfect for remote access (NoMachine/SSH)"
        echo "   ✗ HDMI monitor will NOT display anything"
        echo ""
        echo "💡 Use this when: Working remotely without monitor"
        echo ""
    else
        echo "🖥️  NVIDIA DRIVER (Hardware Display Mode)"
        echo "   ✓ HDMI monitor works perfectly"
        echo "   ✓ Full GPU hardware acceleration"
        echo "   ✓ NoMachine works with monitor connected"
        echo ""
        echo "💡 Use this when: Working with physical display"
        echo ""
    fi

    echo "═══════════════════════════════════════════════════════"
    echo ""

    if [ "$CURRENT_DRIVER" = "dummy" ]; then
        echo "Switch to NVIDIA DRIVER?"
        echo ""
        print_warning "This will restart your display session"
        echo ""
        read -p "Continue? (y/N): " confirm

        if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
            print_info "Switching to NVIDIA driver..."

            if [ -f /usr/share/X11/xorg.conf.d/20-dummy.conf ]; then
                sudo rm /usr/share/X11/xorg.conf.d/20-dummy.conf
            fi

            print_success "Configuration removed (persists across reboots)"
            print_info "Restarting display manager..."
            echo ""
            read -p "Press Enter to restart..."
            sudo systemctl restart gdm3
        fi
    else
        echo "Switch to DUMMY DRIVER?"
        echo ""
        print_warning "This will restart your display session"
        echo ""
        read -p "Continue? (y/N): " confirm

        if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
            print_info "Switching to DUMMY driver..."

            sudo tee /usr/share/X11/xorg.conf.d/20-dummy.conf > /dev/null << 'EOF'
Section "Monitor"
    Identifier "DummyMonitor"
    HorizSync 28.0-80.0
    VertRefresh 48.0-75.0
    Modeline "1920x1080" 172.80 1920 2040 2248 2576 1080 1081 1084 1118
EndSection

Section "Device"
    Identifier "DummyDevice"
    Driver "dummy"
    VideoRam 256000
EndSection

Section "Screen"
    Identifier "DummyScreen"
    Device "DummyDevice"
    Monitor "DummyMonitor"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Modes "1920x1080"
    EndSubSection
EndSection
EOF

            print_success "Configuration created (persists across reboots)"
            print_info "Restarting display manager..."
            echo ""
            read -p "Press Enter to restart..."
            sudo systemctl restart gdm3
        fi
    fi

    echo ""
    read -p "Press Enter to continue..."
}

# Function to quickly enter Isaac Lab container
enter_isaac_lab() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "   Isaac Lab - Quick Access"
    echo "═══════════════════════════════════════════════════════"
    echo ""

    LAB_STATUS=$(check_isaac_lab_status)

    if [ "$LAB_STATUS" = "running" ]; then
        print_success "Container is running"
        echo ""
        print_info "Entering container..."
        echo ""
        cd "${HOME}/docker/isaac-lab/IsaacLab"
        python3 docker/container.py enter
        cd - > /dev/null
    elif [ "$LAB_STATUS" = "stopped" ]; then
        print_warning "Container exists but is stopped"
        echo ""
        print_info "Starting container first..."
        echo ""
        cd "${HOME}/docker/isaac-lab/IsaacLab"
        python3 docker/container.py start
        if [ $? -eq 0 ]; then
            print_success "Container started"
            echo ""
            print_info "Entering container..."
            echo ""
            python3 docker/container.py enter
        fi
        cd - > /dev/null
    else
        print_error "Container not running"
        echo ""
        print_info "Isaac Lab containers are removed when stopped (by design)"
        print_info "Your data is safe in persistent volumes"
        echo ""
        print_info "Use Option 8 to start Isaac Lab container"
        echo ""
        read -p "Press Enter to continue..."
    fi
}

# Function to stop Isaac Lab container
stop_isaac_lab() {
    clear
    echo "═══════════════════════════════════════════════════════"
    echo "   Isaac Lab - Stop Container"
    echo "═══════════════════════════════════════════════════════"
    echo ""

    LAB_STATUS=$(check_isaac_lab_status)

    if [ "$LAB_STATUS" = "running" ]; then
        print_warning "Stopping Isaac Lab container..."
        echo ""
        cd "${HOME}/docker/isaac-lab/IsaacLab"
        python3 docker/container.py stop
        if [ $? -eq 0 ]; then
            echo ""
            print_success "Container stopped successfully"
            echo ""
            print_info "Data is preserved in Docker volumes"
            print_info "Use Option 9 to resume later"
        else
            print_error "Failed to stop container"
        fi
        cd - > /dev/null
    elif [ "$LAB_STATUS" = "stopped" ]; then
        print_info "Container is already stopped"
    else
        print_info "No Isaac Lab container exists"
    fi

    echo ""
    read -p "Press Enter to continue..."
}

# Main menu
main_menu() {
    while true; do
        clear
        echo "═══════════════════════════════════════════════════════"
        echo "         Isaac Sim 5.1.0 & Isaac Lab Launcher"
        echo "           (Official NVIDIA Commands)"
        echo "═══════════════════════════════════════════════════════"
        echo ""

        # Check system status
        CURRENT_DRIVER=$(check_display_driver)
        SIM_STATUS=$(check_isaac_sim_status)
        LAB_STATUS=$(check_isaac_lab_status)

        # Display mode status
        if [ "$CURRENT_DRIVER" = "dummy" ]; then
            print_warning "Display Mode: DUMMY (Headless) - No HDMI output"
        else
            print_success "Display Mode: NVIDIA (Hardware) - HDMI active"
        fi

        # Isaac Sim status
        if [ "$USING_CUSTOM" = true ]; then
            print_success "Isaac Sim: CUSTOM image (with your packages)"
        else
            print_info "Isaac Sim: OFFICIAL image (packages are temporary)"
        fi
        if [ "$SIM_STATUS" = "running" ]; then
            echo -e "${GREEN}✓${NC} Isaac Sim: Container ${GREEN}RUNNING${NC}"
        else
            echo -e "${BLUE}ℹ${NC} Isaac Sim: Container ${YELLOW}STOPPED${NC}"
        fi

        # Isaac Lab container status
        if [ "$LAB_STATUS" = "running" ]; then
            echo -e "${GREEN}✓${NC} Isaac Lab: Container ${GREEN}RUNNING${NC}"
        elif [ "$LAB_STATUS" = "stopped" ]; then
            echo -e "${YELLOW}⚠${NC} Isaac Lab: Container ${YELLOW}STOPPED${NC}"
        else
            echo -e "${BLUE}ℹ${NC} Isaac Lab: Container ${BLUE}not created yet${NC}"
        fi
        echo ""

        # Show setup section if things are missing
        echo "─────────── SETUP (First Time) ─────────────────────────"
        echo -n "s) NGC Login (for pulling images)        "
        echo -e "${YELLOW}[manual check]${NC}"

        echo -n "p) Pull Isaac Sim 5.1.0 Image (~15GB)    "
        if [ "$IMAGE_AVAILABLE" = true ]; then
            echo -e "${GREEN}[FOUND]${NC}"
        else
            echo -e "${RED}[NOT FOUND]${NC}"
        fi

        echo -n "c) Clone Isaac Lab Repository            "
        if [ "$ISAAC_LAB_AVAILABLE" = true ]; then
            echo -e "${GREEN}[FOUND]${NC}"
        else
            echo -e "${RED}[NOT FOUND]${NC}"
        fi
        echo ""

        echo "0) Toggle Display Driver (Dummy ↔ NVIDIA)"
        echo ""
        echo "─────────── ISAAC SIM (Simulation Only) ───────────────"
        if [ "$SIM_STATUS" = "running" ]; then
            echo -e "1) Isaac Sim: Start with GUI - ${GREEN}RUNNING${NC}"
        else
            echo "1) Isaac Sim: Start with GUI"
        fi
        if [ "$SIM_STATUS" = "running" ]; then
            echo -e "2) Isaac Sim: Start Headless - ${GREEN}RUNNING${NC}"
        else
            echo "2) Isaac Sim: Start Headless"
        fi
        echo "3) Isaac Sim: Run Compatibility Check"
        echo "4) Isaac Sim: Setup/Reset Directories"
        echo "5) Isaac Sim: System Information"
        echo "6) Isaac Sim: Check Prerequisites"
        if [ "$SIM_STATUS" = "running" ]; then
            echo -e "7) Isaac Sim: Save Changes (Commit Image) ⭐ - ${GREEN}RUNNING${NC}"
        else
            echo -e "7) Isaac Sim: Save Changes (Commit Image) ⭐ - ${YELLOW}Not Running${NC}"
        fi
        echo ""
        echo "─────────── ISAAC LAB (RL Training) ────────────────────"
        if [ "$LAB_STATUS" = "running" ]; then
            echo -e "8) Isaac Lab: Start/Build Container - ${GREEN}RUNNING${NC}"
        else
            echo "8) Isaac Lab: Start/Build Container ⭐"
        fi

        if [ "$LAB_STATUS" = "running" ]; then
            echo -e "9) Isaac Lab: Quick Access (if running) - ${GREEN}RUNNING${NC}"
        else
            echo -e "9) Isaac Lab: Quick Access (if running) - ${BLUE}Use Option 8 First${NC}"
        fi

        if [ "$LAB_STATUS" = "running" ]; then
            echo -e "a) Isaac Lab: Stop Container - ${GREEN}RUNNING${NC}"
        elif [ "$LAB_STATUS" = "stopped" ]; then
            echo -e "a) Isaac Lab: Stop Container - ${YELLOW}Already Stopped${NC}"
        else
            echo -e "a) Isaac Lab: Stop Container - ${BLUE}Not Available${NC}"
        fi
        echo ""
        echo "─────────── UTILITIES ──────────────────────────────────"
        echo "b) Python Package Management Guide"
        echo ""
        echo "x/q) Exit"
        echo ""
        echo "═══════════════════════════════════════════════════════"
        echo ""
        read -p "Select option [s,p,c,0-9,a,b,x,q]: " choice

        case $choice in
            s|S)
                ngc_login
                ;;
            p|P)
                pull_isaac_sim_image
                ;;
            c|C)
                clone_isaac_lab
                ;;
            0)
                toggle_display_driver
                ;;
            1)
                run_with_gui
                ;;
            2)
                run_headless
                ;;
            3)
                run_compatibility_check
                ;;
            4)
                setup_directories
                read -p "Press Enter to continue..."
                ;;
            5)
                show_system_info
                ;;
            6)
                check_prerequisites
                read -p "Press Enter to continue..."
                ;;
            7)
                save_container_changes
                ;;
            8)
                run_isaac_lab_official
                ;;
            9)
                enter_isaac_lab
                ;;
            a|A)
                stop_isaac_lab
                ;;
            b|B)
                manage_packages
                ;;
            x|q|X|Q)
                echo ""
                print_info "Thank you for using Isaac Sim & Isaac Lab!"
                echo ""
                exit 0
                ;;
            *)
                print_error "Invalid option"
                sleep 1
                ;;
        esac
    done
}

# Script entry point
echo ""
print_info "Isaac Sim 5.1.0 Launcher (Official NVIDIA Commands)"
echo ""

check_prerequisites

main_menu
