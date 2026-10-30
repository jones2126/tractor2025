#!/usr/bin/env bash
# Fail-closed Approval B placeholder. Live motion requires a new authorized revision.
set -euo pipefail

APPROVED_FOR_FIELD=false
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VERIFY="${HERE}/verify_approval_b_package_20261030.py"

if [[ "${1:-}" == "--verify-only" && $# -eq 1 ]]; then
    python3 "${VERIFY}"
    echo "Verification complete. No service, controller, logger, or motion command was started."
    exit 0
fi

echo "DRAFT - UNTESTED IN THE FIELD" >&2
echo "APPROVED_FOR_FIELD=${APPROVED_FOR_FIELD}" >&2
echo "Approval B live start is disabled pending a separate test-day authorization." >&2
echo "Use --verify-only for the non-motion package check." >&2
exit 1
