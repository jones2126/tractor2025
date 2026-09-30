# Phone-to-NAS voice note proof of concept

This is a standalone field test. It records the phone microphone in Firefox or
Chrome, sends the live audio over an encrypted ZeroTier connection to RPi5NAS,
and has the NAS send that audio to Google Cloud Speech-to-Text. A transcript is
saved only when it contains **“Tractor note”**.

It does **not** import mission software, read telemetry, connect to the tractor,
or send commands. It has no Start Mission, Pause, steering, transmission, or
robot-control endpoint.

## What happens to the audio

1. Android records mono Opus audio in a WebM or Ogg container using
   `MediaRecorder`, according to browser support.
2. The browser sends roughly 250 ms pieces over a secure WebSocket to RPi5NAS.
3. RPi5NAS forwards those pieces in memory to Google's streaming Speech-to-Text
   service using the Google Python client and gRPC.
4. The NAS retains transcripts only for phrases that use the wake phrase. It
   never writes raw audio to disk.
5. Google says streaming and synchronous audio is processed in memory and is
   not stored unless the Cloud project is deliberately enrolled in data
   logging. Do **not** opt this test project into data logging.

The phone never receives a Google credential. The permanent Google credential
stays on RPi5NAS outside the Git repository.

## Why this audio and HTTPS design

The page checks `MediaRecorder.isTypeSupported()` before preferring
`audio/webm;codecs=opus` and falling back to `audio/ogg;codecs=opus`; it does not
assume every browser has the same container. Google supports both `WEBM_OPUS`
and `OGG_OPUS` at 8, 12, 16, 24, or 48 kHz, and the Android browser path normally
produces 48 kHz Opus. The configuration therefore uses 48 kHz without
transcoding. Google streaming recognition is gRPC-only, so the browser uses a
normal secure WebSocket to the NAS and the NAS performs the gRPC call.

Microphone access requires a secure browser context. This setup uses a tiny
private certificate authority (CA), with a certificate valid for the known NAS
hostnames and its three listed IP addresses. Only a phone on the private
ZeroTier network can reach the default listener. Installing the CA on your own
phone makes HTTPS trusted without publishing the service or accepting browser
warning pages.

Official references used for this design:

- [MDN: `getUserMedia()` security and permission behavior](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia)
- [MDN: checking MediaRecorder formats](https://developer.mozilla.org/en-US/docs/Web/API/MediaRecorder/isTypeSupported_static)
- [Chrome for Developers: MediaRecorder and secure origins](https://developer.chrome.com/blog/mediarecorder)
- [Mozilla: Firefox for Android permissions](https://support.mozilla.org/en-US/kb/how-firefox-android-use-permissions-it-requests)
- [Google: Chrome for Android microphone permissions](https://support.google.com/chrome/answer/2693767?co=GENIE.Platform%3DAndroid&hl=en)
- [Google Cloud supported audio encodings](https://cloud.google.com/speech-to-text/docs/encoding)
- [Google Cloud streaming recognition](https://cloud.google.com/speech-to-text/docs/streaming-recognize)
- [Google Cloud streaming quotas and limits](https://cloud.google.com/speech-to-text/docs/quotas)
- [Google Cloud Speech data usage FAQ](https://cloud.google.com/speech-to-text/docs/v1/data-usage-faq)

## Files this test creates

Nothing is installed or generated until you run the setup commands.

| Purpose | Location on RPi5NAS |
| --- | --- |
| Program | `/home/al/repos/tractor2025/field_testing/tools/voice_note_poc/` |
| Private settings | `/home/al/.config/tractor-voice-notes/voice-note.env` |
| Google JSON key | `/home/al/.config/tractor-voice-notes/google-service-account.json` |
| Private CA and server TLS files | `/home/al/.config/tractor-voice-notes/tls/` |
| CSV and JSONL logs | `/home/al/field_logs/voice_note_poc/` |

The configuration, Google key, TLS private keys, virtual environment, logs, and
raw audio are not committed to Git. Raw audio is not created on disk at all.

## Part 1 — prepare Google Cloud

Billing must be enabled on the Google Cloud project. Google documents a free
quota, but a billing account is still required; current prices and free usage
can change, so check the [official pricing page](https://cloud.google.com/speech-to-text/pricing)
before field use.

These are actions you perform in your Google account. Do not paste the
downloaded credential into chat.

1. Open the [Google Cloud console](https://console.cloud.google.com/).
2. At the top of the page, open the project selector and choose **New Project**.
   A name such as `tractor-voice-note-test` is fine.
3. Open **Billing** and link the new project to your billing account.
4. Open **APIs & Services → Library**. Search for **Cloud Speech-to-Text API**,
   open it, and select **Enable**.
5. Do not enable the optional Speech-to-Text data-logging program.
6. Open **IAM & Admin → Service Accounts** and select **Create service account**.
   Use a name such as `tractor-voice-note-nas`.
7. Give this service account only **Cloud Speech Client**
   (`roles/speech.client`). It needs recognition access, not Owner or Editor.
8. Open the new service account, select **Keys → Add key → Create new key →
   JSON**, and download the file once.

Google recommends short-lived credentials instead of service-account keys when
an outside workload can use Workload Identity Federation. This NAS does not
already have a suitable external identity provider, so federation would add a
second authentication service just for this small private test. A narrowly
permissioned key stored only on the NAS is the practical POC choice. Revisit
federation before treating this as permanent infrastructure.

Relevant Google instructions:

- [Set up Speech-to-Text and billing](https://cloud.google.com/speech-to-text/docs/setup)
- [Cloud Speech Client role](https://cloud.google.com/iam/docs/roles-permissions/speech)
- [Create a service-account key](https://cloud.google.com/iam/docs/keys-create-delete)
- [Application Default Credentials](https://cloud.google.com/docs/authentication/provide-credentials-adc)

## Part 2 — place the Google credential on RPi5NAS

Use a terminal on RPi5NAS. Replace `~/Downloads/the-file-google-gave-you.json`
with the actual downloaded filename. These commands keep the key outside the
repository and make it readable only by user `al`.

```bash
mkdir -p /home/al/.config/tractor-voice-notes
chmod 700 /home/al/.config/tractor-voice-notes
mv ~/Downloads/the-file-google-gave-you.json \
  /home/al/.config/tractor-voice-notes/google-service-account.json
chmod 600 /home/al/.config/tractor-voice-notes/google-service-account.json
```

If the download happened on another computer, copy it directly to that final
path using a private transfer method, then delete the extra downloaded copy.

## Part 3 — create private HTTPS files

On RPi5NAS:

```bash
cd /home/al/repos/tractor2025/field_testing/tools/voice_note_poc
bash setup_private_https.sh
```

The script refuses to overwrite an existing CA. It creates:

- `tractor-voice-root-ca.crt`: public root certificate; this is the one file to
  copy to the Android phone.
- `root-ca.key`: private CA key; never copy it off the NAS.
- `server.crt` and `server.key`: certificate used by this server.

The certificate covers `192.168.193.217`, `192.168.1.2`, `192.168.1.205`,
`RPi5NAS`, `rpi5nas`, and `rpi5nas.local`. Regenerate it deliberately if those
addresses change.

### Install the public root certificate on Android

Copy only this file to the phone, by USB or another private method:

```text
/home/al/.config/tractor-voice-notes/tls/tractor-voice-root-ca.crt
```

Android menu wording varies by phone version. On a recent Pixel, start with:

1. Open **Settings**.
2. Open **Security & privacy → More security settings → Encryption &
   credentials**.
3. Choose **Install a certificate → CA certificate**. If the exact wording is
   absent, search Android Settings for **Install certificates**.
4. Confirm the Android security message, select
   `tractor-voice-root-ca.crt`, and give it a recognizable name such as
   `Tractor Voice Notes Private CA`.
5. Close and reopen the browser.

Android warns that a user-installed CA could inspect network traffic. That is
why the CA private key stays on your NAS and this CA is used only for this test.
Remove this CA from **Trusted credentials → User** after the experiment if you
do not plan to use it again. See Google's [Pixel certificate instructions](https://support.google.com/pixelphone/answer/2844832)
and [Android network settings](https://support.google.com/android/answer/9654714).

Current Firefox for Android normally uses third-party CAs installed in the
Android certificate store. Mozilla documents this as the default. If Firefox
still reports an untrusted issuer after a full restart, update Firefox first,
then check Firefox **Settings → Privacy & Security** for the option to trust
third-party root certificates. Do not use **Accept the risk and continue** as a
permanent workaround. See [Mozilla's CA guidance](https://support.mozilla.org/en-US/kb/setting-certificate-authorities-firefox).

## Part 4 — configure and launch the service

On RPi5NAS:

```bash
cd /home/al/repos/tractor2025/field_testing/tools/voice_note_poc
cp .env.example /home/al/.config/tractor-voice-notes/voice-note.env
chmod 600 /home/al/.config/tractor-voice-notes/voice-note.env
bash run.sh
```

The first launch creates a local Python virtual environment and downloads the
three dependencies. Later launches reuse it. The last command is the
one-command development launcher.

It listens only on the NAS ZeroTier address by default. Leave that terminal
open during the test. Do not deploy this directory to the tractor Pi.

## Part 5 — use it from the phone

1. Confirm ZeroTier is connected on both the phone and RPi5NAS.
2. In Firefox for Android, open:

   `https://192.168.193.217:8443/`

3. The page should open with a normal secure connection and no certificate
   warning. If Android or Firefox also asks for local-network access, allow it
   for this private NAS test.
4. Tap **START LISTENING** once.
5. When asked, allow microphone access while visiting the site.
6. Wait for **LISTENING** and say, “Tractor note, move one foot left.”
7. The page should show **NOTE SAVED**, sound a short beep, display the note,
   and increase the counter.
8. Say an ordinary sentence without “Tractor note.” It can appear under
   **Last heard**, but it must not increase the saved count.
9. Tap **STOP LISTENING** when finished.

Keep the browser in the foreground for the first test. Chrome's Android help
explicitly notes that a site cannot keep recording when another Chrome tab or
app is active. Android can also suspend background pages to save power. The page
requests a screen wake lock while listening, when the browser supports it.

## Check the saved logs

On RPi5NAS:

```bash
ls -l /home/al/field_logs/voice_note_poc/
tail -n 5 /home/al/field_logs/voice_note_poc/voice_notes_*.csv
tail -n 5 /home/al/field_logs/voice_note_poc/voice_notes_*.jsonl
```

Each accepted record contains:

- unique note ID;
- server UTC and America/New_York timestamps;
- latest client timestamp and seconds since Start;
- full recognized phrase and text following the wake phrase;
- browser user agent, session ID, and connection ID;
- Google confidence when supplied; and
- browser audio MIME type.

Both daily files use the same note ID, so records can be matched exactly.

## Local tests without Google

The core tests need only the Python standard library:

```bash
cd /home/al/repos/tractor2025/field_testing/tools/voice_note_poc
python3 -m unittest discover -s tests -v
```

They cover ordinary-speech rejection, complete and split wake phrases, wake
timeout, duplicate suppression, New York timestamps, and matching CSV/JSONL.

To test HTTPS and the page without spending Google quota, temporarily set
`VOICE_NOTE_FAKE_TRANSCRIPTS=1` in `voice-note.env` and launch normally. The
microphone and WebSocket will run, but audio is discarded. You can test the log
path from another NAS terminal with:

```bash
curl --cacert /home/al/.config/tractor-voice-notes/tls/tractor-voice-root-ca.crt \
  -H 'Content-Type: application/json' \
  -d '{"transcript":"Tractor note move one foot left","session_id":"local-test"}' \
  https://192.168.193.217:8443/api/fake-transcript
```

Set `VOICE_NOTE_FAKE_TRANSCRIPTS=0` and restart before the phone acceptance test.

## Automatic restarts during a long session

Google limits one streaming recognition connection to about five minutes. The
page deliberately starts a fresh browser recorder and WebSocket after 270
seconds. This makes a new WebM header and leaves margin below Google's limit.
The session ID and on-screen count stay the same. A brief gap can occur at that
boundary; this is acceptable for the POC and should be measured in field tests.

If Wi-Fi/cellular/ZeroTier or Google connectivity drops, the page shows
**OFFLINE — RETRYING**, closes the unusable audio stream, and reconnects with
exponential backoff. It does not retain outage audio, because retaining it
would conflict with the default no-raw-audio rule. Speak the note again after
**LISTENING** returns.

When you press **STOP LISTENING**, the page can remain on **TRANSCRIBING** for up
to eight seconds. This intentionally gives Google time to finalize the last
utterance, save it, and update the phone's note counter before the microphone
connection closes.

## Optional systemd service for later

Use this only after the manual test works. It is intentionally not installed or
enabled automatically.

```bash
sudo cp voice-note-poc.service.example /etc/systemd/system/voice-note-poc.service
sudo systemctl daemon-reload
sudo systemctl enable --now voice-note-poc.service
sudo systemctl status voice-note-poc.service
```

To stop and disable it:

```bash
sudo systemctl disable --now voice-note-poc.service
```

## Troubleshooting

### The page will not open

- Confirm the phone and NAS both show connected in ZeroTier.
- From the NAS, run `ip address` and confirm `192.168.193.217` still belongs to
  the ZeroTier interface.
- Confirm `bash run.sh` is still running and says it is listening on port 8443.
- Use the exact `https://` address, including `:8443`.
- Firefox for Android may separately ask permission to contact local-network
  devices. Allow the NAS for this site.

### Certificate warning remains

- Confirm you installed `tractor-voice-root-ca.crt`, not `server.crt`.
- Confirm the address is one of the IPs or names listed in Part 3.
- Fully close and reopen the browser after installing the CA.
- In Firefox, update the app and confirm third-party root trust is enabled.
- Never install `root-ca.key` or `server.key` on the phone.
- Do not proceed through a warning page as the long-term fix.

### MICROPHONE BLOCKED

- Firefox: Android **Settings → Apps → Firefox → Permissions → Microphone →
  Allow while using**. Then clear the site's microphone decision and reload.
- Chrome: **Chrome → Settings → Site settings → Microphone**, open the NAS site,
  and select Allow. Also check Android's Chrome microphone permission.
- Make sure another app is not exclusively using a Bluetooth headset microphone.

### Format is “Unsupported”

Update Firefox or Chrome. Try Chrome as the fallback. The code intentionally
does not send a format Google cannot decode. Record the browser version and
phone model if only one browser fails.

### Google credentials are not configured

- Check that the JSON file exists at the path in `voice-note.env`.
- Run `stat /home/al/.config/tractor-voice-notes/google-service-account.json`;
  permissions should be `0600` and owner should be `al`.
- Never paste the file contents into the terminal history, source code, or chat.

### Google says permission denied or API disabled

- Confirm the Cloud Speech-to-Text API is enabled in the same project named in
  the JSON credential.
- Confirm billing is linked and active.
- Confirm the service account has **Cloud Speech Client**, not merely a role on
  the service account itself.
- New service-account keys can take a minute to become usable.

### Listening restarts or stops after silence

The page reconnects after network or streaming failures and proactively renews
the Google stream every 270 seconds. Keep the page visible. If Android battery
saving still suspends it, disable battery optimization for the test browser for
the duration of a controlled test, then restore the setting afterward.

### Wake phrase or note is split

The server carries `tractor`, `tractor note`, and the next final recognition
fragment across boundaries for eight seconds. It also ignores a normalized
duplicate note in the same session for 15 seconds. Save the browser, headset,
noise, and exact phrase details if this still fails; those observations will
guide the continuous-field version.

### Running tractor or wind noise causes poor recognition

- First test parked with the engine off.
- Then compare phone microphone versus a close Bluetooth headset microphone.
- Keep the microphone out of direct wind.
- Use the saved confidence values only as a diagnostic; Google does not always
  return confidence and says it should not be treated as certainty.
- Test engine idle, working RPM, wind, Wi-Fi/cellular handoff, Firefox, and
  Chrome as separate test cases so the cause is clear.

## Acceptance checklist

- [ ] Page loads at trusted `https://192.168.193.217:8443/` over ZeroTier.
- [ ] Android grants microphone access.
- [ ] One tap changes the display to LISTENING.
- [ ] “Tractor note, move one foot left” produces NOTE SAVED and a beep.
- [ ] CSV and JSONL contain `move one foot left` under the same note ID.
- [ ] Ordinary speech does not change the saved count.
- [ ] Stop releases the microphone.
- [ ] Repeat once in Chrome if Firefox shows a browser-specific problem.
