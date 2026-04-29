# Start with an NVIDIA CUDA 13.0 image
FROM nvidia/cuda:13.0.0-devel-ubuntu22.04

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install core ZED SDK dependencies
RUN apt-get update && apt-get install -y \
    python3 python3-pip \
    libusb-1.0-0-dev \
    libhidapi-libusb0 \
    libcrypto++-dev \
    libboost-all-dev \
    zstd wget libpng-dev \
    libgoogle-glog-dev \
    libfreeimage-dev \
    libatlas-base-dev \
    libopenblas-dev \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy the exact SDK installer and your receiver code
COPY ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.1.2.zstd.run /app/zed_sdk_installer.run
COPY zed_receiver.cpp /app/

# Run the installer
# We use -- nrc (no runtime check) to skip hardware checks during build
RUN chmod +x zed_sdk_installer.run && \
    ./zed_sdk_installer.run -- silent skip_cuda skip_python nrc


# Compile the receiver with stub linking
# Create placeholder stubs for the video libraries so the linker is satisfied
# Copy the existing CUDA stub to fake the video and encode libraries
RUN cp /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libnvcuvid.so && \
    cp /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libnvidia-encode.so && \
    cp /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libnvcuvid.so.1 && \
    cp /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libnvidia-encode.so.1

# Now compile (same command as before)
RUN g++ -O3 zed_receiver.cpp -o zed_receiver \
    -I/usr/local/zed/include \
    -I/usr/local/cuda/include \
    -L/usr/local/zed/lib \
    -L/usr/local/cuda/lib64 \
    -L/usr/local/cuda/lib64/stubs \
    -Wl,--allow-shlib-undefined \
    -lsl_zed -lcudart -lcuda -lboost_system -lpthread -lrt

CMD ["./zed_receiver"]