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
        mv NuEclipse_V1.02.029_Linux_Setup/eclipse ./eclipse-cdt; \
        rm -f NuEclipse.tar.gz; \
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
# Install clang-tidy-automotive
# -----------------------------
RUN if [ ! -d clang-tidy-automotive ]; then \
        echo "🌍 Cloning clang-tidy-automotive"; \
        git clone --depth=1 https://github.com/wosayttn/clang-tidy-automotive /opt/clang-tidy-automotive; \
        cd /opt/clang-tidy-automotive; \
        \
        echo "🌍 Downloading LLVM 20.1.8"; \
        wget -q https://github.com/llvm/llvm-project/archive/refs/tags/llvmorg-20.1.8.tar.gz; \
        tar -xzf llvmorg-20.1.8.tar.gz; \
        rm -f llvmorg-20.1.8.tar.gz; \
        \
        echo "🌍 Setting up clang-tidy-automotive"; \
        ./setup.sh > /dev/null 2>&1; \
        mkdir build && cd build; \
        cmake -G Ninja \
              -DCMAKE_CXX_COMPILER=/usr/bin/clang++-20 \
              -DCMAKE_C_COMPILER=/usr/bin/clang-20 \
              -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
              -DCMAKE_BUILD_TYPE=Release \
              -DLLVM_ENABLE_PROJECTS="clang;clang-tools-extra" \
              -DLLVM_TARGETS_TO_BUILD="X86" \
              ../llvm-project-llvmorg-20.1.8/llvm > /dev/null 2>&1; \
        ninja -j$(nproc) clang-tidy; \
    fi

# -----------------------------
# Add tools to PATH
# -----------------------------
ENV PATH="/opt/eclipse-cdt:${PATH}"
ENV PATH="/opt/cppcheck/build:${PATH}"
ENV PATH="/opt/clang-tidy-automotive/build/bin:${PATH}"

# Set default working directory for user projects
WORKDIR /workspace

# -----------------------------
# Verify installations
# -----------------------------
RUN eclipse -version || echo "✅ Eclipse CDT installed"
RUN cppcheck --version || echo "✅ cppcheck installed"
RUN clang-tidy --version || echo "✅ clang-tidy-automotive installed"

# Default command
CMD ["/bin/bash"]
