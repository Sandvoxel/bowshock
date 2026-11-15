#!/usr/bin/env bash
# Usage: run.sh <file.ork> <outdir>   — runs the OpenRocket reference harness quietly.
set -euo pipefail
JAR="${OPENROCKET_JAR:-/Applications/OpenRocket.app/Contents/Resources/app/OpenRocket.jar}"
HERE="$(cd "$(dirname "$0")" && pwd)"
java -Djava.awt.headless=true -cp "$JAR" "$HERE/Harness.java" "$1" "$2" 2>&1 | grep -v -E "DEBUG|INFO|^WARNING:|warning:|^\s+\^|Application.getMotorSetDatabase" || true
