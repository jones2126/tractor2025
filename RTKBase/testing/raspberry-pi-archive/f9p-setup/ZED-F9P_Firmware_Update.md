# ZED‑F9P Firmware Update Procedure

_Last updated: 2025‑09_

This guide explains how to check the current firmware version of a u‑blox ZED‑F9P receiver and update it to the latest release (HPG 1.51 as of Sept 2025).

---

## 1. Check Current Firmware Version

1. Connect your F9P to u‑center via USB and open **View → Messages View** (or press **F9**).
2. Expand **UBX → MON (Monitor)** in the tree on the left.
3. Select **VER (Version)** and click the **Poll** button (paper airplane icon).
4. Look for the line `FWVER=HPG x.xx` or `EXT CORE x.xx` in the right‑hand pane.
   - Example: `FWVER=HPG 1.32` → running firmware 1.32.

---

## 2. Download the Latest Firmware

1. Visit the u‑blox resources page for ZED‑F9P:  
   https://www.u-blox.com/en/product/zed-f9p-module?legacy=Current#Documentation-&-resources
2. Download the latest `*.bin` firmware file .  
   - As of Sept 2025: **HPG 1.51** is the latest recommended release.

---

## 3. Prepare for Update

- **Backup configuration:** In u‑center, go to `View → Messages View → CFG (Configuration)` and click **Save current configuration** to back up BBR/Flash settings.
- **Use a stable power supply:** Firmware update takes 1–2 minutes; interruption can brick the module.

---

## 4. Perform the Update

1. In u‑center: `Tools → Firmware Update` (or `Receiver → Firmware Update` depending on version).
2. In the Firmware Update Tool:
   - **File:** Browse to the downloaded `.bin` file.
   - **Protocol:** Keep at **u-blox**.
   - **Port:** Ensure correct COM port is selected.
   - Leave other fields default unless instructed by release notes.
3. Click the green “GO” button in the lower left corner. Progress bar will show flashing and verification.
4. When complete, the F9P will reboot automatically.

---

## 5. Verify the Update

Repeat **Step 1** (MON‑VER Poll) and confirm `FWVER=HPG 1.51` (or the version you installed).

---

## 6. Restore Configuration

If settings were reset:
- Reload saved config (`CFG → Load configuration` from BBR/Flash file).
- Verify port assignments, output messages, and RTCM configuration.
- Save to Flash again.

---

## 7. Troubleshooting

| Problem | Fix |
|--------|------|
| Update tool fails immediately | Ensure correct COM port, baud rate (115200), and exclusive access (close other apps using port). |
| Update stalls | Try lower baud rate (9600) and re-run update. |
| Device unresponsive after failure | Retry update with "Use Safe Boot" jumper on F9P (see HW manual). |

---

## References

- ZED‑F9P Product Page: https://www.u-blox.com/en/product/zed-f9p-module  
- HPG 1.51 Release Notes: https://content.u-blox.com/sites/default/files/documents/ZED-F9P-FW100HPG151_RN_UBXDOC-963802114-13110.pdf  
- Firmware Update Tool (in u‑center): https://www.u-blox.com/en/product/u-center

