#!/usr/bin/env bash
# wgpu_draft.sh: the first result of the draft-first flow (the Draft drag table of the example rocket, `engine3d_draft_first` with BOWSHOCK_DRAFT_ONLY) on another backend or device
# than CUDA on the default GPU (DESIGN3D item 57):
#   tools/gpuq/wgpu_draft.sh wgpu     the Vulkan (wgpu) backend of the default NVIDIA GPU: what the backend costs on the same silicon
#   tools/gpuq/wgpu_draft.sh igpu     the integrated AMD GPU of the local host (RADV, the NVIDIA ICD hidden from the Vulkan loader), under wgpu
# One `DEVICE` line, the `RUN` lines and the `DRAFT-FIRST first result` line are the output.
set -eu
case "${1:?wgpu|igpu}" in
  wgpu) export LUMA_BACKEND=wgpu ;;
  igpu) export LUMA_BACKEND=wgpu VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/radeon_icd.x86_64.json ;;
  *) echo "usage: wgpu_draft.sh wgpu|igpu" >&2; exit 2 ;;
esac
export BOWSHOCK_ENGINE=3d BOWSHOCK_DRAFT_ONLY=1
cargo nextest run --release -p bowshock engine3d_draft_first --run-ignored only --no-capture
