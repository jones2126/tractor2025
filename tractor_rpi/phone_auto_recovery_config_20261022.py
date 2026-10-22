"""Shared timing settings for phone-requested AUTO link recovery.

Change these values together with the associated deterministic tests before a
field deployment.  The 500 ms motion watchdog is intentionally separate and
must not be increased to create a longer recovery window.
"""

PHONE_HEARTBEAT_TIMEOUT_S = 0.5
AUTO_LINK_RECOVERY_WINDOW_S = 30.0
AUTO_LINK_STABLE_SECONDS = 5.0
