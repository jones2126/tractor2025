# 2026-10-19 AXE75 Proton log-delivery prompt

## Ready-to-paste prompt

```text
Help me establish durable automatic log collection from the TractorField
TP-Link Archer AXE75 without exposing credentials or creating an open mail
relay.

Read:

- obsidian_vault/02-testing/20261006-master-manual-phone-rerun-network-failure.md
- obsidian_vault/02-testing/20261019-tractor01-testing-regrouping.md
- RTKBase/setup/README.md

Inspect `git status` before editing and preserve unrelated work.

Context:

- The AXE75's local address is `192.168.10.1`.
- The wired RTK-base Pi 3 was `192.168.10.114`.
- The router's volatile log was lost when it was intentionally restarted.
- The AXE75 supports automatic SMTP mail logs.
- Proton Mail Bridge is running on the RPi5NAS and shows a local SMTP endpoint
  at `127.0.0.1:1025` using STARTTLS.

Critical networking fact: `127.0.0.1` always means the device making the
connection. Entering `127.0.0.1` in the AXE75 would point the router back to
itself, not to Proton Bridge on the NAS. A Python send test on the NAS validates
the Bridge credentials and TLS behavior only; it does not prove that the field
router can reach that endpoint.

Do not put any Proton, Gmail, router, or mail password/token in source code,
chat output, git, logs, screenshots, or generated artifacts. Use interactive
prompts, environment variables for an ephemeral test, or a root-owned mode-0600
credential file outside the repository. Redact SMTP dialogue.

Please do the following:

1. On the NAS, build a minimal one-time Python SMTP smoke test for
   `127.0.0.1:1025` with STARTTLS and authentication. Send only a harmless test
   message to an address I specify. Verify receipt and record only pass/fail,
   timestamps, TLS/auth method, and sanitized error information.
2. Inspect how Proton Bridge binds its SMTP listener and certificate. Do not
   make Bridge directly LAN- or Internet-visible merely to satisfy the router.
3. Compare safe delivery architectures:
   - direct Proton SMTP submission token, if my Proton plan and address are
     eligible;
   - a tightly restricted local relay on the wired RTK-base Pi that accepts
     only the AXE75 source address and forwards through an authenticated,
     encrypted path to a NAS relay/Bridge;
   - a dedicated Gmail sender using a revocable app password;
   - periodic supported router mail plus independent Pi network telemetry.
4. Lead with a recommendation based on security, behavior when Starlink or
   ZeroTier is down, ability to survive router reboot, operational complexity,
   and how quickly evidence is preserved.
5. If implementing a relay, bind it only to the necessary interface, firewall
   it to the router, require or strictly constrain authentication/source, set
   message and rate limits, prevent arbitrary recipients, rotate logs, and
   prove from an unapproved host that relay is rejected. Never expose an
   unauthenticated SMTP port to the Internet or ZeroTier broadly.
6. Configure or document the AXE75 only after the selected end-to-end path is
   proven. Test manual mail, scheduled mail, router restart, WAN outage and
   recovery, clock synchronization, duplicate delivery, and log truncation.
7. Separately design a persistent Pi 3 network sentinel that records router
   reachability/latency, Tractor01 reachability, ARP/MAC changes, DHCP/lease
   state, Ethernet carrier, router restart windows, and control/dashboard port
   probes. Router email is not a replacement for this independent record.
8. Add deployment, verification, rollback, credential-rotation, archive, and
   shutdown documentation. Do not install or modify services until I approve
   the recommended architecture.

Begin with the local NAS-to-Bridge smoke-test plan and the architecture
comparison. Ask me only for choices that materially affect the design; never
ask me to paste a secret into chat.
```
