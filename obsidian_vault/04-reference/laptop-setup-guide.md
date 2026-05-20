# New Laptop Setup Guide — tractor2025 Dev Environment

**Platform:** Ubuntu (desktop/laptop), AMD GPU
**Script:** `laptop-setup.sh` in this same folder — run it after reading this guide.

---

## 1. System Prerequisites

```bash
sudo apt update && sudo apt install -y curl git
```

---

## 2. AMD GPU Driver Fix (Radeon Mullins APU only)

Without this, the desktop UI hangs under load. Skip if you don't have AMD Radeon Mullins hardware.

Edit GRUB to force `amdgpu` driver instead of `radeon`:

```bash
sudo nano /etc/default/grub
```

Find the line:
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
```

Change it to:
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash amdgpu.cik_support=1 radeon.cik_support=0"
```

Apply and reboot:
```bash
sudo update-grub
sudo reboot
```

After reboot, verify:
```bash
lsmod | grep -E 'amdgpu|radeon'
# amdgpu should show >100 users, radeon should show 0
```

---

## 3. Sudo Without Password (optional convenience)

Lets you run `sudo` without typing your password. Security trade-off — only on personal machines.

```bash
echo 'albert ALL=(ALL) NOPASSWD: ALL' | sudo tee /etc/sudoers.d/albert-nopasswd
```

Replace `albert` with your actual username if different.

---

## 4. Obsidian

Installs the latest release automatically:

```bash
sudo apt update && sudo apt install -y curl
TAG=$(curl -s https://api.github.com/repos/obsidianmd/obsidian-releases/releases/latest \
  | grep '"tag_name"' | cut -d'"' -f4)
VERSION=${TAG#v}
DEB_URL="https://github.com/obsidianmd/obsidian-releases/releases/download/${TAG}/obsidian_${VERSION}_amd64.deb"
curl -L "$DEB_URL" -o /tmp/obsidian.deb
sudo apt install -y /tmp/obsidian.deb
```

After install, open Obsidian and set vault:
**Open folder as vault → `/home/albert/tractor2025/obsidian_vault/`**

---

## 5. Claude Code

Install via npm (requires Node.js) or follow current instructions at claude.ai/code.

```bash
claude --version    # verify install
claude doctor       # check environment
```

Start Claude Code from the project directory:
```bash
cd ~/claude-robot-project   # legacy vault (existing sessions)
# or
cd ~/tractor2025            # new vault location
claude
```

---

## 6. NoMachine (Remote Desktop)

Download the `.deb` from [nomachine.com](https://www.nomachine.com) then:

```bash
sudo apt install ./nomachine_<version>_amd64.deb
```

Note: `rpm -i` does NOT work on Ubuntu — must use `apt install ./filename.deb`.

---

## 7. ZeroTier (VPN for robot network)

```bash
curl -s https://install.zerotier.com | sudo bash
sudo zerotier-cli join 9f77fc393e0a16f8
```

After joining, approve the device at [my.zerotier.com](https://my.zerotier.com).

Verify connectivity:
```bash
ip address                  # find your ZeroTier IP (192.168.193.x range)
ping 192.168.193.76        # ping the tractor RPi
```

Key ZeroTier addresses:
| Device | ZeroTier IP |
|--------|-------------|
| Tractor RPi 5 | 192.168.193.76 |
| Bridgeville RTK Base | 192.168.193.88 |

---

## 8. PuTTY (SSH client with GUI)

```bash
sudo apt install -y putty
```

**Known issue — PuTTY crashes on launch with "unable to load font server:fixed"**

Fix: Before first launch, open PuTTY settings and change the font:
- Window → Appearance → Font → Change to `client:Ubuntu Mono 12`

If PuTTY won't open at all, launch it with the X11 backend workaround:
```bash
GDK_BACKEND=x11 putty
```
Then immediately fix the font setting so you don't need this workaround again.

Common SSH targets:
```bash
ssh al@192.168.1.151        # tractor RPi (local network)
ssh al@192.168.193.76       # tractor RPi (ZeroTier)
```

---

## 9. Clone Repositories

```bash
cd ~
git clone https://github.com/jones2126/tractor2025.git
```

The `obsidian_vault/` folder inside `tractor2025/` is automatically included.

---

## 10. PlatformIO (Teensy firmware compilation)

Not captured in setup history but required for firmware work:

```bash
# Install PlatformIO Core (CLI)
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
python3 get-platformio.py

# Add to PATH (add this to ~/.bashrc)
export PATH=$PATH:~/.platformio/penv/bin
```

Compile and upload tractor firmware:
```bash
cd ~/tractor2025/tractor_teensy
pio run --target upload
pio device monitor
```

---

## What's Not Covered Here

- ROS2 installation (not currently in use)
- OAK-D / DepthAI SDK (must stay at version 2.30.0.0)
- Python virtual environments for RPi scripts
- Fusion 360 (Windows/Mac only)

---

## See Also

- [[00-project-overview]] — hardware reference, RPi connection, key file paths
- `tractor_rpi/install_services.sh` — RPi-side service installation
