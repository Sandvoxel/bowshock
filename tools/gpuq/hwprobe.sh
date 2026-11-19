#!/usr/bin/env bash
# hwprobe.sh: what the GPU of this host is and whether its clocks can be lowered without sudo (DESIGN3D item 57): prints the device, the clock and power ranges, tries `nvidia-smi -lgc`
# and `-pl` as the plain user (each undone at once, the original limit restored), lists the Vulkan adapters wgpu sees.
nvidia-smi --query-gpu=name,driver_version,memory.total,clocks.max.sm,clocks.max.mem,power.limit,power.min_limit,power.max_limit --format=csv
echo "--- lgc as $(whoami)"
nvidia-smi -lgc 1200,1200 2>&1 | head -2
nvidia-smi -rgc 2>&1 | head -1
echo "--- pl"
pl=$(nvidia-smi --query-gpu=power.limit --format=csv,noheader,nounits | head -1 | tr -d ' ')
nvidia-smi -pl "${pl%.*}" 2>&1 | head -2
echo "--- vulkan adapters"
vulkaninfo --summary 2>&1 | grep -E "deviceName|driverName|deviceType" | head -12
echo "--- host"
nproc; free -g | head -2
