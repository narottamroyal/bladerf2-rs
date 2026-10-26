#!/usr/bin/env bash
# Lets Chromium use the bladeRF 2.0 micro from the browser test origin without
# a permission prompt, by installing a managed WebUSB policy.
#
# Usage:
#   sudo scripts/browser-policy.sh install   # write the policy
#   sudo scripts/browser-policy.sh remove    # delete it
#
# Linux only, for Chromium. The policy applies to every Chromium profile on the
# machine, but only to the origin below.
set -euo pipefail

# Must match WASM_BINDGEN_TEST_ADDRESS in .cargo/config.toml.
ORIGIN="http://127.0.0.1:8123"
VENDOR_ID=11504   # 0x2CF0, Nuand
PRODUCT_ID=21072  # 0x5250, bladeRF 2.0 micro
POLICY_DIR=/etc/chromium/policies/managed
POLICY_FILE="$POLICY_DIR/bladerf2-webusb.json"

if [[ "$(id -u)" -ne 0 ]]; then
    echo "run as root: sudo $0 ${1:-install}" >&2
    exit 1
fi

case "${1:-}" in
    install)
        mkdir -p "$POLICY_DIR"
        cat > "$POLICY_FILE" <<EOF
{
  "WebUsbAllowDevicesForUrls": [
    {
      "devices": [{"vendor_id": $VENDOR_ID, "product_id": $PRODUCT_ID}],
      "urls": ["$ORIGIN"]
    }
  ]
}
EOF
        python3 -m json.tool "$POLICY_FILE" > /dev/null
        echo "installed $POLICY_FILE"
        ;;
    remove)
        rm -f "$POLICY_FILE"
        echo "removed $POLICY_FILE"
        ;;
    *)
        echo "usage: sudo $0 install|remove" >&2
        exit 2
        ;;
esac
