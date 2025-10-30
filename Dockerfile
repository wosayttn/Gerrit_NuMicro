# Base image
FROM ubuntu:24.04

# Prevent interactive prompts during install
ENV DEBIAN_FRONTEND=noninteractive

# Install essential packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    clang-20 \
    cppcheck \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-venv \
    wget \
    unzip \
    default-jre \
    make \
    unzip \ 
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /opt

# Download and install Eclipse CDT
RUN if [ ! -d eclipse-cdt ]; then \
        echo "🌍 Downloading Eclipse CDT"; \
        wget -q -O NuEclipse.tar.gz --no-check-certificate https://www.nuvoton.com/resource-download.jsp?tp_GUID=SW132023052507200264; \
        tar -xzf NuEclipse.tar.gz; \
        mv NuEclipse_V1.02.029_Linux_Setup/eclipse ./eclipse-cdt; \
        rm -f NuEclipse.tar.gz; \
    fi

# Set default working directory for builds
WORKDIR /workspace

# Default command
CMD ["/bin/bash"]
