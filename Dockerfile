# Use a Ubuntu 22.04 base image
FROM ubuntu:22.04

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update apt and install the required packages
RUN apt-get update && apt-get install -y \
    # - build-essential: Contains gcc, g++, and make
    build-essential \
    # - cmake: The build system
    cmake \
    # - git: For cloning repositories if needed
    git \
    # - wget: For downloading files
    wget \
    # - unzip: For extracting compressed files
    unzip \
    # - libopencv-dev: OpenCV library
    libopencv-dev \
    # - libeigen3-dev: Eigen library
    libeigen3-dev \
    # - libyaml-cpp-dev: For parsing your .yaml config files
    libyaml-cpp-dev \
    # -  Python3 and related packages for matplotlib
    python3 \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install python packages via pip
RUN pip3 install --no-cache-dir \
    pykitti \
    numpy \
    matplotlib \
    scipy \
    opencv-python-headless

# Set the working directory inside the container
WORKDIR /app

# Copy the local project files into the container
COPY . /app

# Create a build directory
RUN mkdir -p build

# Run cmake to configure the project
RUN cd build && \
    cmake ..

# Default command
CMD ["/bin/bash"]