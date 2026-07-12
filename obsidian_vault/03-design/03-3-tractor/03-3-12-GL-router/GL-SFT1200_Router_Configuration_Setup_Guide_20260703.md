# GL-SFT1200 Router — Configuration Setup Guide

**Purpose:** general setup steps for a GL-SFT1200 in this project - from
factory-default to operational, covering the web admin panel (GUI) parts.
For the CLI/SSH side (deploying `wifi_publish.sh`, its known quirks, and
day-to-day testing), see the other three docs in this set:
- `wifi_publish_sh_Setup_Guide_20260703.md` - deploying the monitoring script
- `WiFi_Signal_Monitoring_Test_Quickref_20260703.md` - day-to-day testing
- `GLiNet_router_wifi_monitoring_setup_20260703.md` - deep reference on
  this hardware's SSH/scp/nc quirks

This doc covers the GUI side those three don't: first-time setup, joining
an upstream WiFi network (Repeater/WISP mode), broadcasting the local
`TractorField` network, and static DHCP reservations.

---

## 1. First-time access

1. Connect a device to the router - either its LAN port via Ethernet, or
   its WiFi SSID (printed on a sticker on the bottom of the unit).
2. Open a browser to `http://192.168.8.1`.
3. Choose a display language.
4. **Set an admin password** - there is no default password; you must set
   one (minimum 5 characters) before the panel unlocks. This is separate
   from any WiFi password and only controls the admin panel + underlying
   Linux system.

**Known quirk:** if the browser keeps redirecting to the raw LuCI
interface (`.../cgi-bin/luci`) instead of the friendlier GL.iNet panel,
try `http://192.168.8.1/index` directly instead of the bare IP.

---

## 2. Join an upstream WiFi network (Repeater / WISP mode)

This is how the router gets its own internet/network connection - by
connecting to another WiFi network, rather than its own separate WAN
cable. By default this runs in **WISP mode**: the router creates its own
subnet and firewalls itself from the upstream network - which is exactly
the behavior the two-subnet design in the field-network runbook relies on.

1. In the admin panel, go to **INTERNET → Repeater**.
2. Click **Connect** - the router scans for nearby networks automatically.
3. Choose the target SSID from the list (e.g. `Pixel_4952`, or another
   corner's `TractorField` network, depending on this router's role).
4. Enter the password, click **Apply**.
5. A green dot appears next to **Repeater** on the INTERNET page once
   connected successfully.

**If the target SSID doesn't appear in the scan list:** the upstream
network may be using a WiFi channel this router doesn't support - the
Repeater page shows which channels it can see; if the target network is
outside that list, it won't show up no matter how close you are.

**If connecting to a phone hotspot specifically:** keep the phone's screen
unlocked during the scan - many phones stop transmitting WiFi beacon
frames reliably once the screen locks, which can make the hotspot
invisible to the scan even though it's technically "on."

---

## 3. Configure the local broadcast network (`TractorField`)

This is the separate WiFi network this router broadcasts *for the
tractor to join* - independent from whatever it's using upstream in
Step 2.

1. Go to the **WIRELESS** section of the admin panel.
2. Select the relevant band (2.4 GHz or 5 GHz, per whichever radio this
   deployment calls for).
3. Click **Modify**, set:
   - SSID: `TractorField` (or `TractorField-5G` if distinguishing bands
     across corners, per the field-network runbook's naming convention)
   - Password: match whatever the tractor router's client config expects
   - Channel: leave on Auto unless a specific channel is required to
     avoid interference with a neighboring radio
4. Apply.

**Note if this router is also in Repeater mode (Step 2) on the same
band:** the channel for the local broadcast network gets fixed to
whatever channel the upstream network is using, when both roles share a
band - not independently selectable. This matters if the upstream network
(the hotspot) changes bands between sessions, which has been observed to
happen unpredictably on this hardware - see the radio-contention note in
`Outdoor_Field_Network_CPE210_Implementation_Runbook.md`, Section 4.

---

## 4. Static DHCP reservation for a known device

This is how a device gets a *fixed* IP every time it connects, instead of
whatever the DHCP pool happens to hand out - what was set up for
`tractor02` earlier in this project.

1. In the admin panel, find the connected client (usually under **Clients**
   or the LAN/DHCP section - exact label can vary slightly by firmware
   version).
2. Add a static lease binding the device's MAC address to the IP you want
   it to always receive.
3. **The client device must reconnect** (or the router's DHCP lease must
   expire and renew) before the new static assignment takes effect - it
   won't apply retroactively to an already-active connection.

**If you need to find a device's current IP/MAC to reserve it:** on the
device itself:
```bash
ip a show eth0
```
gives both the current IP and MAC address directly.

---

## 5. SSH / CLI access

The admin panel has a toggle for enabling remote SSH access to the
router's command line - needed for anything in the other three docs
(deploying `wifi_publish.sh`, running `iwinfo`, checking `ps`, etc.).
Exact menu location varies slightly by firmware version - look under
**System** or **More Settings** for an SSH/remote-access toggle if it's
not already enabled.

**Once SSH is reachable, every connection from a client machine needs
compatibility flags this router's SSH server requires** (covered in full
in `GLiNet_router_wifi_monitoring_setup_20260703.md`):
```bash
ssh -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa root@<router-ip>
```
and for `scp`, additionally:
```bash
scp -O -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa ...
```

---

## 6. What's configured through the GUI vs. the CLI, at a glance

| Task | Where |
|---|---|
| Admin password, first-time setup | GUI |
| Joining an upstream WiFi network (Repeater/WISP) | GUI |
| Local broadcast SSID/password (`TractorField`) | GUI |
| Static DHCP reservations | GUI |
| Enabling SSH access itself | GUI |
| Checking which radio matches a given SSID (`iwinfo`) | CLI (SSH) |
| Deploying/running `wifi_publish.sh` | CLI (SSH/SCP) |
| Setting up `wifi_publish.sh` as a persistent boot service | CLI (SSH) |
| Forcing WiFi reassociation (`wifi` command, `wpa_cli reconnect`) | CLI (SSH) |
