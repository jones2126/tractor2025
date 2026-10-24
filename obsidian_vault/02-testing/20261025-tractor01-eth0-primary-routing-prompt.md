# New chat prompt - configure Tractor01 eth0 as the normal underlay

Use this prompt in a new field chat and attach or direct the chat to the
companion context document:

`obsidian_vault/02-testing/20261025-tractor01-eth0-primary-routing-context.md`

## Ready-to-paste prompt

```text
I am in the field with Tractor01 and a Windows 10 laptop using PuTTY. Help me
configure and verify Tractor01 so its persistent normal physical network path
is:

default via 192.168.8.1 dev eth0 src 192.168.8.20

Read the attached context document completely before directing any change:

obsidian_vault/02-testing/20261025-tractor01-eth0-primary-routing-context.md

Treat the context as evidence and constraints, not as proof of the Pi's live
state. Start with read-only discovery. Identify the actual network manager,
exact connection-profile names, addresses, routes, metrics, ZeroTier peers and
physical underlay, service state, and systemd drop-ins. Do not assume names or
silently reuse historical values.

Important requirements:

- Keep the tractor parked, engine off, transmission physically disengaged,
  cutting deck off, and phone control in Pause.
- This is network configuration only. Do not start a mission, enable Auto,
  authorize motion, or change firmware/controller behavior.
- The desired path is Pi eth0 -> physical Ethernet cable -> GL-SFT1200 LAN
  192.168.8.1 -> GL upstream Wi-Fi -> AXE75/internet.
- The Pi's onboard wlan0 must not automatically connect directly to the AXE75
  TractorField SSID during mission operation.
- Route metric preference alone is not sufficient while wlan0 retains a
  directly connected 192.168.10.0/24 route.
- Do not disconnect wlan0 until the eth0/GL path has independently demonstrated
  gateway, Internet, DNS, ZeroTier, PuTTY, and RTK-base reachability.
- Protect against remote lockout with saved rollback commands, a short timed
  automatic rollback, and a second session/local recovery path where possible.
- Make only persistent, minimal changes. Preserve unrelated Wi-Fi profiles and
  repository work.
- Inspect and remove the temporary runtime tractorfield-mission systemd
  override if it still exists, but do not start a motion-authorized service as
  part of route testing.
- Never expose an operator key in chat or saved output.

Walk me through this one short command group at a time. After each group,
interpret my output and tell me whether it is safe to continue. Do not give me
one large script to run blindly.

Before changing anything, record a complete before-state. Before cancelling
rollback, prove all of the following:

1. eth0 has 192.168.8.20/24.
2. the persistent default route is through 192.168.8.1 on eth0.
3. `ip route get 1.1.1.1` reports `dev eth0 src 192.168.8.20`.
4. wlan0 is not associated with TractorField.
5. the GL gateway, Internet, and DNS are stable through eth0.
6. ZeroTier is online, PuTTY reconnects to 192.168.193.76, and the encrypted
   ZeroTier underlay is demonstrated to leave through eth0 rather than wlan0.
7. the ZeroTier RTK base at 192.168.193.88 remains reachable through the wired
   uplink.

Save before/after evidence. Ask me before rebooting. After an approved reboot,
repeat the checks and run a stationary network soak. If anything is ambiguous
or the wired uplink cannot independently carry the required traffic, stop and
use the documented rollback rather than improvising.

At completion, summarize the exact persistent changes, verification evidence,
remaining risks, and rollback commands. Do not describe the tractor as ready
for a mission. A later mission still requires the Heading-F9P volatile-RAM
configuration and independent readback, rtcm-server restart and heading
reacquisition, and a fresh full MISSION PREFLIGHT PASS.
```
