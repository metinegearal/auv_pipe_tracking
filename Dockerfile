FROM osrf/ros:jazzy-desktop

# Prevent interactive prompts during apt installations
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies, computer vision libraries, and GTSAM requirements
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    libgtsam-dev \
    libsuitesparse-dev \
    libmetis-dev \
    libopencv-dev \
    nlohmann-json3-dev \
    ros-jazzy-behaviortree-cpp \
    && rm -rf /var/lib/apt/lists/*

# Fix the Ubuntu 24.04 GTSAM CppUnitLite packaging bug permanently
RUN cd /tmp && \
    touch dummy.cpp && \
    g++ -c dummy.cpp && \
    ar rcs libCppUnitLite.a dummy.o && \
    cp libCppUnitLite.a /usr/lib/x86_64-linux-gnu/ && \
    rm dummy.* libCppUnitLite.a

# Install HoloOcean and Python requirements
# RUN pip3 install --no-cache-dir setuptools==58.2.0 holoocean --break-system-packages && \
#     pip3 install --no-cache-dir --upgrade setuptools --break-system-packages

# Strip the private git link from requirements, install the local HoloOcean, then the rest
# RUN sed -i -e '/byu-holoocean/d' -e '/numpy/d' /tmp/requirements.txt && \
#     pip3 install --no-cache-dir setuptools==58.2.0 --break-system-packages && \
#     pip3 install --no-cache-dir --ignore-installed "numpy<2.0.0" --break-system-packages && \
#     pip3 install --no-cache-dir /tmp/HoloOcean/client --break-system-packages && \
#     pip3 install --no-cache-dir -r /tmp/requirements.txt --break-system-packages && \
#     pip3 install --no-cache-dir --upgrade setuptools --break-system-packages

# --- NEW PIP INSTALLATION BLOCK ---
COPY requirements.txt /tmp/requirements.txt
COPY HoloOcean /tmp/HoloOcean

# Remove the private git link, then install everything cleanly
RUN sed -i '/byu-holoocean/d' /tmp/requirements.txt && \
    pip3 install --no-cache-dir setuptools==58.2.0 --break-system-packages && \
    pip3 install --no-cache-dir /tmp/HoloOcean/client --break-system-packages && \
    pip3 install --no-cache-dir --upgrade setuptools --break-system-packages && \
    pip3 install --no-cache-dir --ignore-installed kiwisolver "numpy<2.0.0" --break-system-packages && \
    pip3 install --no-cache-dir --pre torch torchvision --index-url https://download.pytorch.org/whl/nightly/cu126 --break-system-packages && \
    pip3 install --no-cache-dir -r /tmp/requirements.txt --break-system-packages
# ----------------------------------
# Create workspace directory
WORKDIR /workspace

ARG USER_UID=1000
# --- NEW: Create non-root user for Unreal Engine ---
RUN (userdel -r ubuntu || true) && \
    useradd -m -u ${USER_UID} -s /bin/bash auv_user && \
    usermod -aG sudo,video auv_user && \
    echo "auv_user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chown -R auv_user:auv_user /workspace

# Switch to the new user
USER auv_user

# Source ROS 2 and local workspace automatically for the new user
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc

CMD ["/bin/bash"]