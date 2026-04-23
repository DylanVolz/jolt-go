#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

examples=(
  "./example"
  "./examples/character_virtual"
  "./examples/contact_events"
  "./examples/path_constraint"
  "./examples/soft_body_cloth"
  "./examples/state_recorder"
)

for pkg in "${examples[@]}"; do
  echo "==> go run ${pkg}"
  go run "${pkg}"
  echo ""
done

echo "All examples completed."
