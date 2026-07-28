#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${1:-/tmp/sentry_time_alignment_variants}"
SOURCE_CONFIG="${ROOT_DIR}/src/aim_solver/config/aim_solver.yaml"

mkdir -p "${OUT_DIR}"

make_variant() {
  local name="$1"
  local front_offset="$2"
  local front_1_offset="$3"
  local back_offset="$4"
  local output="${OUT_DIR}/${name}.yaml"

  cp "${SOURCE_CONFIG}" "${output}"
  sed -i     -e "s/^    front_tf_timestamp_offset_sec: .*/    front_tf_timestamp_offset_sec: ${front_offset} # generated variant/"     -e "s/^    front_1_tf_timestamp_offset_sec: .*/    front_1_tf_timestamp_offset_sec: ${front_1_offset} # generated variant/"     -e "s/^    back_tf_timestamp_offset_sec: .*/    back_tf_timestamp_offset_sec: ${back_offset} # generated variant/"     "${output}"
  printf '%s\n' "${output}"
}

make_variant fixed_default -0.024 -0.026 -0.024
make_variant fixed_zero 0.0 0.0 0.0
make_variant fixed_minus_20ms -0.020 -0.020 -0.020
make_variant fixed_minus_25ms -0.025 -0.025 -0.025
make_variant fixed_minus_30ms -0.030 -0.030 -0.030
make_variant fixed_minus_35ms -0.035 -0.035 -0.035
