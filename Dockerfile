##############################################################
# Stage 1: Builder — compile cppcheck + clang-tidy-automotive
##############################################################
FROM ubuntu:24.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git file wget unzip \
        clang-20 ninja-build python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Build cppcheck
# -------------------------
RUN git clone --branch 2.18.3 --depth 1 https://github.com/danmar/cppcheck.git && \
    cd cppcheck && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

# -------------------------
# Build clang-tidy-automotive
# -------------------------
RUN git clone --depth=1 https://github.com/wosayttn/clang-tidy-automotive /opt/clang-tidy && \
    cd /opt/clang-tidy && \
    wget -q https://github.com/llvm/llvm-project/archive/refs/tags/llvmorg-20.1.8.tar.gz && \
    tar -xzf llvmorg-20.1.8.tar.gz && rm llvmorg-20.1.8.tar.gz && \
    ./setup.sh > /dev/null 2>&1 && \
    mkdir build && cd build && \
    cmake -G Ninja \
        -DCMAKE_CXX_COMPILER=/usr/bin/clang++-20 \
        -DCMAKE_C_COMPILER=/usr/bin/clang-20 \
        -DCMAKE_BUILD_TYPE=Release \
        -DLLVM_ENABLE_PROJECTS="clang;clang-tools-extra" \
        -DLLVM_TARGETS_TO_BUILD="X86" \
        ../llvm-project-llvmorg-20.1.8/llvm > /dev/null 2>&1 && \
    ninja -j$(nproc) clang-tidy

##############################################################
# Stage 2: Final Runtime Image — ultra small
##############################################################
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt

# Install only runtime tools (NO compilers, NO build tools)
RUN apt-get update && apt-get install -y --no-install-recommends \
        git file python3 python3-pip python3-setuptools python3-wheel \
        default-jre wget unzip \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Install Eclipse CDT + ARM toolchains
# -------------------------
RUN wget -q -O NuEclipse.tar.gz --no-check-certificate \
        "https://www.nuvoton.com/resource-download.jsp?tp_GUID=SW132023052507200264" \
    && tar -xzf NuEclipse.tar.gz \
    && mv NuEclipse_V1.02.029_Linux_Setup/eclipse ./eclipse-cdt \
    && tar -xf NuEclipse_V1.02.029_Linux_Setup/Others/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.xz -C /opt \
    && tar -xf NuEclipse_V1.02.029_Linux_Setup/Others/gcc-arm-10.3-2021.07-x86_64-aarch64-none-elf.tar.xz -C /opt \
    && rm -rf NuEclipse* *.tar* *.xz

# -------------------------
# Copy cppcheck from builder
# -------------------------
COPY --from=builder /opt/cppcheck/build /opt/cppcheck

# Copy clang-tidy-automotive binary
COPY --from=builder /opt/clang-tidy/build/bin/clang-tidy /opt/clang-tidy/bin/

# -------------------------
# PATH
# -------------------------
ENV PATH="/opt/eclipse-cdt:${PATH}"
ENV PATH="/opt/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux/bin:${PATH}"
ENV PATH="/opt/gcc-arm-10.3-2021.07-x86_64-aarch64-none-elf/bin:${PATH}"
ENV PATH="/opt/cppcheck:${PATH}"
ENV PATH="/opt/clang-tidy/bin:${PATH}"

WORKDIR /workspace

CMD ["/bin/bash"]
