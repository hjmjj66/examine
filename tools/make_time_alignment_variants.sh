#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${1:-/tmp/sentry_time_alignment_variants}"
SOURCE_CONFIG="${ROOT_DIR}/src/aim_solver/config/aim_solver.yaml"

mkdir -p "${OUT_DIR}"

make_variant() {
  local name="$1"
  local offset="$2"
  local adaptive="$3"
  local output="${OUT_DIR}/${name}.yaml"

  cp "${SOURCE_CONFIG}" "${output}"
  sed -i \
    -e "s/^    tf_timestamp_offset_sec: .*/    tf_timestamp_offset_sec: ${offset} # generated variant/" \
    -e "s/^    enable_time_alignment_estimator: .*/    enable_time_alignment_estimator: ${adaptive}/" \
    "${output}"
  printf '%s\n' "${output}"
}

make_variant fixed_zero 0.0 false
make_variant fixed_minus_20ms -0.020 false
make_variant fixed_minus_25ms -0.025 false
make_variant fixed_minus_30ms -0.030 false
make_variant fixed_minus_35ms -0.035 false
make_variant adaptive_minus_25ms_seed -0.025 true
