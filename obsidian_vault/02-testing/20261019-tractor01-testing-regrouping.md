# 2026-10-19 Tractor01 testing regrouping

## Purpose

This is the launch point for separate, narrowly scoped chats. The October 6
field incident, coverage-planning work, and historical mission rerun should not
be combined into one implementation thread.

The existing dirty worktree contains unrelated boundary and coverage work.
Every new chat must begin with `git status`, preserve those changes, and avoid
rewriting or deleting them.

## Recommended order

1. **Phone-control recovery and network architecture** — desk/bench work first.
   Decide whether ZeroTier can meet the safety and timing requirements for
   primary motion authority; reproduce page-hidden Pause and recovery without a
   moving tractor.
2. **Manual Recovery to Parked Position** — turn the successful October 6
   improvised recovery into a preinstalled, Manual-only, guarded procedure.
3. **AXE75 log delivery** — validate SMTP locally, then select a safe way for
   the field router to reach a mail relay. This can proceed independently of
   mission geometry.
4. **Staged coverage planning** — replace the one-shot black box with explicit
   owner review gates. Do not proceed past a gate without review.
5. **September master-route rerun** — prepare the immutable route and field
   checklist only after the control/recovery work has passed its bench gates.

Do not combine deliberate network-fault injection with the long historical
mission. First prove fault and recovery behavior in stationary, engine-off and
wheels-raised tests, then in a separately approved short low-speed exercise.

## Evidence corrections to carry forward

- At 10:11:09 on October 6, the hidden phone page intentionally selected Pause.
- At 10:13:34, guarded Auto reached Tractor01.
- After the five-second gate, the controller resumed driving commands at
  10:13:40.
- The JRK target changed from neutral `2836` to forward near `2300`; feedback
  reached about `2296` and held until the heartbeat expired at 10:14:01.
  Therefore, the remaining question is why there was no observed movement, not
  whether a forward JRK command was sent.
- Repeated later failures were specific to Tractor01's `wlan0`/TractorField
  path. The independently logged wired RTK-base Pi remained healthy.
- The temporary onboard-router recovery path performed well, but Al's desired
  future hierarchy is ZeroTier primary, phone hotspot fallback, onboard travel
  router Manual recovery, with a possible later hardwired last-resort device.
  This desired hierarchy must be safety-tested; it is not yet an approved
  implementation.

## Separate new-chat prompts

- [[20261019-phone-control-recovery-zerotier-primary-prompt]]
- [[20261019-manual-recovery-to-parked-position-prompt]]
- [[20261019-staged-coverage-planning-prompt]]
- [[20261019-september-master-route-rerun-prompt]]
- [[20261019-axe75-proton-log-delivery-prompt]]

## Suggested focus for today

Start the first two chats and ask them to produce bench-ready changes and
checklists. Start the coverage chat when you want an independent planning task.
Do not start the full route in the field today merely because the historical
package still verifies; the network architecture and recovery gates come first.
