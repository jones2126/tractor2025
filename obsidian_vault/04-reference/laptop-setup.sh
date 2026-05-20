#!/bin/bash
# laptop-setup.sh
# Fresh Ubuntu desktop setup for tractor2025 development.
# Run as your normal user (not root) — sudo will prompt as needed.
# See laptop-setup-guide.md for full notes and manual steps.
#
# Usage: bash laptop-setup.sh

set -e
echo "=== tractor2025 Dev Environment Setup ==="
echo ""

# -------------------------------------------------------------------
# 1. System prerequisites
echo "[1/7] Installing system prerequisites..."
sudo apt update && sudo apt install -y curl git putty

# -------------------------------------------------------------------
# 2. Sudo without password (optional — comment out if not wanted)
echo "[2/7] Configuring passwordless sudo for $USER..."
echo "$USER ALL=(ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/$USER-nopasswd > /dev/null

# -------------------------------------------------------------------
# 3. Obsidian (latest release, auto-detected)
echo "[3/7] Installing Obsidian..."
TAG=$(curl -s https://api.github.com/repos/obsidianmd/obsidian-releases/releases/latest \
  | grep '"tag_name"' | cut -d'"' -f4)
VERSION=${TAG#v}
DEB_URL="https://github.com/obsidianmd/obsidian-releases/releases/download/${TAG}/obsidian_${VERSION}_amd64.deb"
curl -L "$DEB_URL" -o /tmp/obsidian.deb
sudo apt install -y /tmp/obsidian.deb
rm /tmp/obsidian.deb
echo "  Obsidian $VERSION installed."

# -------------------------------------------------------------------
# 4. ZeroTier
echo "[4/7] Installing ZeroTier..."
curl -s https://install.zerotier.com | sudo bash
sudo zerotier-cli join 9f77fc393e0a16f8
echo "  ZeroTier installed. Approve this device at my.zerotier.com"

# -------------------------------------------------------------------
# 5. Clone tractor2025 repo
echo "[5/7] Cloning tractor2025 repository..."
if [ -d "$HOME/tractor2025" ]; then
    echo "  ~/tractor2025 already exists — skipping clone."
else
    git clone https://github.com/jones2126/tractor2025.git "$HOME/tractor2025"
    echo "  Cloned to ~/tractor2025"
fi

# -------------------------------------------------------------------
# 6. PlatformIO
echo "[6/7] Installing PlatformIO..."
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
  -o /tmp/get-platformio.py
python3 /tmp/get-platformio.py
rm /tmp/get-platformio.py

# Add PlatformIO to PATH in .bashrc if not already there
if ! grep -q "platformio/penv/bin" "$HOME/.bashrc"; then
    echo 'export PATH=$PATH:~/.platformio/penv/bin' >> "$HOME/.bashrc"
    echo "  Added PlatformIO to PATH in ~/.bashrc"
fi

# -------------------------------------------------------------------
# 7. AMD GPU fix reminder
echo "[7/7] AMD GPU driver check..."
if lsmod | grep -q "^radeon "; then
    echo ""
    echo "  WARNING: radeon driver is active."
    echo "  For AMD Radeon Mullins APU, switch to amdgpu to prevent UI hangs."
    echo "  See laptop-setup-guide.md Section 2 for manual steps."
else
    echo "  GPU driver OK (radeon not loaded)."
fi

# -------------------------------------------------------------------
echo ""
echo "=== Setup complete ==="
echo ""
echo "Manual steps still required:"
echo "  1. Install NoMachine from nomachine.com (download .deb, then: sudo apt install ./nomachine_*.deb)"
echo "  2. Install Claude Code — see claude.ai/code"
echo "  3. Fix PuTTY font: Window → Appearance → Font → client:Ubuntu Mono 12"
echo "  4. Open Obsidian: Open folder as vault → ~/tractor2025/obsidian_vault/"
echo "  5. Approve ZeroTier device at my.zerotier.com"
echo "  6. Reload PATH:  source ~/.bashrc"
echo ""
echo "AMD GPU fix (if needed): see laptop-setup-guide.md Section 2"
