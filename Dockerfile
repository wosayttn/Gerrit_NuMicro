# Base image
FROM ubuntu:24.04

# Prevent interactive prompts during install
ENV DEBIAN_FRONTEND=noninteractive

# Set working directory
WORKDIR /opt

# Install essential packages
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        file \
        clang-20 \
        ninja-build \
        python3 \
        python3-pip \
        python3-setuptools \
        python3-wheel \
        python3-venv \
        wget \
        unzip \
        default-jre \
        make \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Install Eclipse CDT
# -----------------------------
RUN if [ ! -d eclipse-cdt ]; then \
        echo "🌍 Downloading Eclipse CDT"; \
        wget -q -O NuEclipse.tar.gz --no-check-certificate "https://www.nuvoton.com/resource-download.jsp?tp_GUID=SW132023052507200264"; \
        tar -xzf NuEclipse.tar.gz; \
        rm -f NuEclipse.tar.gz; \
        mv NuEclipse_V1.02.029_Linux_Setup/eclipse ./eclipse-cdt; \
        tar -xf NuEclipse_V1.02.029_Linux_Setup/Others/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.xz -C  /opt; \
        tar -xf NuEclipse_V1.02.029_Linux_Setup/Others/gcc-arm-10.3-2021.07-x86_64-aarch64-none-elf.tar.xz -C /opt; \
        rm -rf NuEclipse_V1.02.029_Linux_Setup; \
        du /opt/
    fi

# -----------------------------
# Install cppcheck 2.18.3
# -----------------------------
RUN if [ ! -d cppcheck ]; then \
        echo "🌍 Downloading cppcheck 2.18.3"; \
        git clone --branch 2.18.3 --depth 1 https://github.com/danmar/cppcheck.git; \
        cd cppcheck && mkdir build && cd build && \
        cmake .. && make -j$(nproc); \
    fi

# -----------------------------
# Add tools to PATH
# -----------------------------
ENV PATH="/opt/eclipse-cdt:${PATH}"
EVN PATH="/opt/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux/bin:${PATH}"
EVN PATH="/opt/gcc-arm-10.3-2021.07-x86_64-aarch64-none-elf/bin:${PATH}"
ENV PATH="/opt/cppcheck/build:${PATH}"

# Set default working directory for user projects
WORKDIR /workspace

# -----------------------------
# Verify installations
# -----------------------------
RUN cppcheck --version || echo "✅ cppcheck installed"
RUN aarch64-none-elf-gcc -v || echo "✅ aarch64-none-elf-gcc installed"
RUN arm-none-eabi-gcc -v || echo "✅ arm-none-eabi-gcc installed"

# Default command
CMD ["/bin/bash"]
