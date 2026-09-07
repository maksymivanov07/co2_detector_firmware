#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cli=${ARDUINO_CLI:-arduino-cli}
if ! command -v "$cli" >/dev/null 2>&1; then
  cli='/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli'
fi
case "${1:-c3}" in
  c3) fqbn='esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=min_spiffs'; target=esp32c3 ;;
  *) echo 'Usage: tools/build.sh [c3] (ESP32-C3 Super Mini)' >&2; exit 2 ;;
esac
"$cli" compile --fqbn "$fqbn" --warnings all --jobs 2 --build-path "$project_dir/build/cache-$target" --output-dir "$project_dir/build/$target" "$project_dir"
