#!/usr/bin/env python3
"""Voice-guidance and hands-free note dashboard for the approved perimeter test."""
from __future__ import annotations
import csv
import importlib.util
import json
import os
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from urllib import error as urllib_error
from urllib import request as urllib_request
from urllib.parse import urlparse

HERE=Path(__file__).resolve().parent
SOURCE=HERE.parents[4]/"tractor_rpi/pure-pursuit/mission_dashboard_20260910.py"
spec=importlib.util.spec_from_file_location("mission_dashboard_base",SOURCE)
dashboard=importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(dashboard)
dashboard.MISSION=HERE/"generated/62_Collins_consolidated_perimeter_1mps_20260929.txt"
dashboard.AUDIT=HERE/"generated/62_Collins_consolidated_perimeter_1mps_20260929_audit.csv"
dashboard.LAUNCHER=HERE/"run_62_Collins_consolidated_perimeter_20260929.sh"
dashboard.EXPECTED_CONFIRMATION="RUN CONSOLIDATED PERIMETER BLADES OFF"

base_safe_to_start=dashboard.safe_to_start
def wifi_safe_to_start(state):
    ok,reason=base_safe_to_start(state)
    if not ok:
        return False,reason.replace("handheld","phone").replace("Handheld","Phone")
    snap=state.snapshot();wifi=snap.get("bridge",{}).get("wifi_control",{})
    if wifi.get("mode")!=0:
        return False,"Put the phone in Pause before starting"
    if wifi.get("heartbeat_fresh")!=1:
        return False,"Phone heartbeat is not fresh; keep the Wi-Fi control page open"
    if wifi.get("estop_latched")!=0:
        return False,"Wi-Fi E-stop is latched; reset it and remain in Pause"
    try: command_age_ms=int(wifi.get("command_age_ms"))
    except (TypeError,ValueError): return False,"Wi-Fi command age is missing"
    if command_age_ms>500:
        return False,f"Phone command is stale ({command_age_ms} ms)"
    return True,""
dashboard.safe_to_start=wifi_safe_to_start

def notify_wifi_dashboard_url(dashboard_url):
    message=(
        "Open this Tractor01 mission dashboard on the laptop. Keep the Wi-Fi "
        "phone control page in the phone's foreground and in Pause; keep the physical e-stop available. This "
        "temporary link includes the operator key.\n\n"+dashboard_url
    )
    request=urllib_request.Request(
        dashboard.NTFY_TOPIC_URL,data=message.encode("utf-8"),
        headers={"Title":"Tractor01 dashboard ready","Click":dashboard_url,"Tags":"tractor"},
        method="POST",
    )
    try:
        with urllib_request.urlopen(request,timeout=5) as response:
            if not 200<=response.status<300: raise RuntimeError(f"ntfy returned HTTP {response.status}")
        print(f"ntfy: preferred dashboard link sent to {dashboard.NTFY_TOPIC_URL}")
    except (urllib_error.URLError,OSError,RuntimeError) as exc:
        print(f"WARNING: could not send dashboard link to ntfy: {exc}")
dashboard.notify_dashboard_url=notify_wifi_dashboard_url

NOTE_DIR=Path(os.environ.get(
    "TRACTOR_VOICE_NOTE_DIR",
    "/home/al/field_logs/20260929_consolidated_perimeter_1mps",
))
NOTE_SESSION=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
NOTE_CSV=NOTE_DIR/f"voice_notes_{NOTE_SESSION}.csv"
NOTE_JSONL=NOTE_DIR/f"voice_notes_{NOTE_SESSION}.jsonl"
NOTE_LOCK=threading.Lock()
NOTE_FIELDS=[
    "note_id","server_timestamp_utc","client_timestamp","transcript",
    "mission_active","process_state","waypoint_index","waypoint_number",
    "route_distance_m","lat","lon","heading_deg","actual_speed_mps",
    "commanded_speed_mps","cross_track_error_m","lookahead_m","fix_quality",
    "heading_carrier","controller_age_s","gps_age_s",
]
with dashboard.AUDIT.open(newline="",encoding="utf-8-sig") as handle:
    AUDIT_ROWS=list(csv.DictReader(handle))


def clean_number(value):
    try: return float(value)
    except (TypeError,ValueError): return None


def record_voice_note(state,payload):
    transcript=" ".join(str(payload.get("transcript","")).split())
    if not transcript: raise ValueError("Voice note transcript is empty")
    if len(transcript)>500: raise ValueError("Voice note transcript exceeds 500 characters")
    snap=state.snapshot();controller=snap.get("controller",{});gps=snap.get("gps",{})
    try: waypoint_index=max(0,int(controller.get("waypoint_idx",0)))
    except (TypeError,ValueError): waypoint_index=0
    audit=AUDIT_ROWS[min(waypoint_index,max(0,len(AUDIT_ROWS)-1))] if AUDIT_ROWS else {}
    lat=clean_number(gps.get("lat"));lon=clean_number(gps.get("lon"));heading=clean_number(gps.get("heading_deg"))
    if lat is None: lat=clean_number(controller.get("lat"))
    if lon is None: lon=clean_number(controller.get("lon"))
    if heading is None: heading=clean_number(controller.get("heading_compass_deg"))
    note={
        "note_id":uuid.uuid4().hex,
        "server_timestamp_utc":datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "client_timestamp":str(payload.get("client_timestamp", "")),
        "transcript":transcript,
        "mission_active":bool(snap.get("mission_active")),
        "process_state":snap.get("process_state"),
        "waypoint_index":waypoint_index,
        "waypoint_number":waypoint_index+1,
        "route_distance_m":clean_number(audit.get("distance_m")),
        "lat":lat,"lon":lon,"heading_deg":heading,
        "actual_speed_mps":clean_number(controller.get("actual_speed_mps")),
        "commanded_speed_mps":clean_number(controller.get("speed_cmd_mps")),
        "cross_track_error_m":clean_number(controller.get("cross_track_err_m")),
        "lookahead_m":clean_number(controller.get("lookahead_dist_m")),
        "fix_quality":controller.get("fix_quality") or gps.get("fix_quality"),
        "heading_carrier":controller.get("heading_carrier") or gps.get("carrier"),
        "controller_age_s":clean_number(snap.get("controller_age_s")),
        "gps_age_s":clean_number(snap.get("gps_age_s")),
    }
    json_record=dict(note)
    json_record["telemetry"]={"controller":controller,"gps":gps,"bridge":snap.get("bridge",{})}
    with NOTE_LOCK:
        NOTE_DIR.mkdir(parents=True,exist_ok=True)
        new_csv=not NOTE_CSV.exists()
        with NOTE_CSV.open("a",newline="",encoding="utf-8") as handle:
            writer=csv.DictWriter(handle,fieldnames=NOTE_FIELDS)
            if new_csv: writer.writeheader()
            writer.writerow({key:note.get(key) for key in NOTE_FIELDS})
        with NOTE_JSONL.open("a",encoding="utf-8",newline="\n") as handle:
            handle.write(json.dumps(dashboard.json_safe(json_record),separators=(",",":"),allow_nan=False)+"\n")
    return note


base_handler_factory=dashboard.handler_factory
def voice_handler_factory(state,token,mission_payload):
    BaseHandler=base_handler_factory(state,token,mission_payload)
    class VoiceHandler(BaseHandler):
        def do_POST(self):
            if urlparse(self.path).path!="/api/note":
                return super().do_POST()
            if not self.authorized():
                self.send_json({"error":"Invalid operator key"},403);return
            length=int(self.headers.get("Content-Length","0"))
            if length>4096:
                self.send_json({"error":"Voice note request is too large"},413);return
            try:
                payload=json.loads(self.rfile.read(length) or b"{}")
                note=record_voice_note(state,payload)
            except (ValueError,OSError) as exc:
                self.send_json({"error":str(exc)},400);return
            self.send_json({"ok":True,"note":note})
    return VoiceHandler
dashboard.handler_factory=voice_handler_factory

replacements={
"Tractor01 — 62 Collins clear-sky resume":"Tractor01 — consolidated perimeter field test",
"Resumes at source waypoint 91. Recovery stays in the current phase and may advance at most 30 m.":"Approved blades-off field test. Voice notes listen for ‘Tractor note…’ while guidance is active.",
"Start the reviewed clear-sky resume mission at source waypoint 91 with blades off?":"Start the approved consolidated perimeter mission with blades off and direct supervision?",
"RUN PARTIAL RINGS BLADES OFF":dashboard.EXPECTED_CONFIRMATION,
"Keep the handheld with you.</b> After a browser Pause: select handheld Pause, press CLEAR PAUSE, confirm HANDHELD PAUSE, then select Auto.":"Keep the phone control page in the phone's foreground.</b> Use this dashboard on the laptop. After a dashboard Pause: select phone Pause, press CLEAR PAUSE, then use guarded Auto only when the route is clear.",
"Keep the handheld in Pause until the controller is ready.":"Keep the phone in Pause until the controller is ready.",
"HANDHELD PAUSE":"PHONE PAUSE",
"Handheld Pause":"Phone Pause",
"Handheld modes":"Control modes",
}
for original,updated in replacements.items():
    if dashboard.HTML.count(original)!=1: raise RuntimeError(f"Dashboard text changed; expected one occurrence of: {original}")
    dashboard.HTML=dashboard.HTML.replace(original,updated)

radio_fact="fact('Radio / steering state',(b.radio?.signal||'—')+' / '+(st.state||'—'))"
wifi_fact="fact('Wi-Fi heartbeat / steering state',(b.wifi_control?.heartbeat_fresh===1?'fresh':'stale')+' / '+(st.state||'—'))"
if dashboard.HTML.count(radio_fact)!=1: raise RuntimeError("Dashboard radio status fact changed")
dashboard.HTML=dashboard.HTML.replace(radio_fact,wifi_fact)

toolbar_old='<button id="guide" class="button guide">START VOICE GUIDANCE</button>'
toolbar_new=toolbar_old+'<span id="noteStatus" class="badge">VOICE NOTES READY</span>'
if dashboard.HTML.count(toolbar_old)!=1: raise RuntimeError("Dashboard voice-guidance button changed")
dashboard.HTML=dashboard.HTML.replace(toolbar_old,toolbar_new)

voice_js=r"""
const noteStatus=document.getElementById('noteStatus');
const SpeechRecognitionAPI=window.SpeechRecognition||window.webkitSpeechRecognition;
let noteRecognition=null,noteWanted=false,noteRunning=false,noteSpeechActive=false,noteRestartTimer=null,noteRestartDelay=500,pendingWakeUntil=0;
function setNoteStatus(text,kind=''){noteStatus.textContent=text;noteStatus.className='badge '+kind}
function scheduleNoteRestart(delay=500){clearTimeout(noteRestartTimer);if(noteWanted&&!noteSpeechActive)noteRestartTimer=setTimeout(startVoiceNotes,delay)}
function startVoiceNotes(){
  noteWanted=true;
  if(!SpeechRecognitionAPI){setNoteStatus('VOICE NOTES UNSUPPORTED','bad');return}
  if(noteRunning||noteSpeechActive)return;
  if(!noteRecognition){
    noteRecognition=new SpeechRecognitionAPI();noteRecognition.continuous=true;noteRecognition.interimResults=false;noteRecognition.lang='en-US';
    noteRecognition.onstart=()=>{noteRunning=true;noteRestartDelay=500;setNoteStatus('LISTENING: SAY TRACTOR NOTE','ok')};
    noteRecognition.onspeechstart=()=>setNoteStatus('HEARING SPEECH','warn');
    noteRecognition.onresult=e=>{for(let i=e.resultIndex;i<e.results.length;i++){if(!e.results[i].isFinal)continue;handleNoteTranscript(e.results[i][0].transcript)}};
    noteRecognition.onerror=e=>{noteRunning=false;if(e.error==='not-allowed'||e.error==='service-not-allowed'){noteWanted=false;setNoteStatus('MICROPHONE PERMISSION BLOCKED','bad')}else if(e.error==='network'){noteRestartDelay=5000;setNoteStatus('VOICE NOTES NEED INTERNET — RETRYING','bad')}else if(e.error!=='aborted'&&e.error!=='no-speech'){noteRestartDelay=1500;setNoteStatus('VOICE ERROR: '+e.error,'bad')}};
    noteRecognition.onend=()=>{noteRunning=false;scheduleNoteRestart(noteRestartDelay)};
  }
  try{noteRecognition.start()}catch(e){if(e.name!=='InvalidStateError'){setNoteStatus('VOICE START FAILED','bad');scheduleNoteRestart(1500)}}
}
function stopVoiceNotes(){noteWanted=false;noteSpeechActive=false;pendingWakeUntil=0;clearTimeout(noteRestartTimer);if(noteRecognition&&noteRunning){try{noteRecognition.stop()}catch(e){}}noteRunning=false;setNoteStatus('VOICE NOTES STOPPED','')}
function pauseVoiceNotesForSpeech(){noteSpeechActive=true;clearTimeout(noteRestartTimer);if(noteRecognition&&noteRunning){try{noteRecognition.stop()}catch(e){}}}
function resumeVoiceNotesAfterSpeech(){noteSpeechActive=false;scheduleNoteRestart(350)}
async function handleNoteTranscript(raw){
  const transcript=String(raw||'').trim(),match=transcript.match(/\btractor\s+note\b[\s,:-]*(.*)$/i);
  let note='';
  if(match){note=match[1].trim();pendingWakeUntil=note?0:Date.now()+8000;if(!note){setNoteStatus('SAY YOUR NOTE','warn');return}}
  else if(Date.now()<pendingWakeUntil){note=transcript;pendingWakeUntil=0}
  else{setNoteStatus('LISTENING: SAY TRACTOR NOTE','ok');return}
  if(!note)return;
  setNoteStatus('SAVING NOTE…','warn');
  try{await api('/api/note','POST',{transcript:note,client_timestamp:new Date().toISOString()});setNoteStatus('NOTE SAVED','ok');speak('Note saved.')}
  catch(e){setNoteStatus('NOTE SAVE FAILED','bad');speak('Voice note was not saved.')}
}
if(!SpeechRecognitionAPI)setNoteStatus('VOICE NOTES UNSUPPORTED','bad');
else if(!window.isSecureContext)setNoteStatus('VOICE NOTES READY — HTTP MAY BLOCK MIC','warn');
"""
anchor="guideBtn.onclick=()=>{"
if dashboard.HTML.count(anchor)!=1: raise RuntimeError("Dashboard guide handler changed")
dashboard.HTML=dashboard.HTML.replace(anchor,voice_js+"\n"+anchor)

old_speak="function speak(text){if(!voiceGuidance||!('speechSynthesis' in window))return;window.speechSynthesis.cancel();const utterance=new SpeechSynthesisUtterance(text);utterance.rate=.9;utterance.volume=1;window.speechSynthesis.speak(utterance)}"
new_speak="function speak(text){if(!voiceGuidance||!('speechSynthesis' in window))return;pauseVoiceNotesForSpeech();window.speechSynthesis.cancel();const utterance=new SpeechSynthesisUtterance(text);utterance.rate=.9;utterance.volume=1;utterance.onend=resumeVoiceNotesAfterSpeech;utterance.onerror=resumeVoiceNotesAfterSpeech;window.speechSynthesis.speak(utterance)}"
if dashboard.HTML.count(old_speak)!=1: raise RuntimeError("Dashboard speak function changed")
dashboard.HTML=dashboard.HTML.replace(old_speak,new_speak)

old_stop="function stopGuidance(message='Voice guidance stopped.'){\n  voiceGuidance=false;"
new_stop="function stopGuidance(message='Voice guidance stopped.'){\n  stopVoiceNotes();voiceGuidance=false;"
if dashboard.HTML.count(old_stop)!=1: raise RuntimeError("Dashboard stop-guidance function changed")
dashboard.HTML=dashboard.HTML.replace(old_stop,new_stop)

old_start="voiceGuidance=true;lastGuidanceAt=0;"
new_start="voiceGuidance=true;startVoiceNotes();lastGuidanceAt=0;"
if dashboard.HTML.count(old_start)!=1: raise RuntimeError("Dashboard start-guidance state changed")
dashboard.HTML=dashboard.HTML.replace(old_start,new_start)

if __name__=="__main__":
    print(f"Hands-free voice-note CSV:   {NOTE_CSV}")
    print(f"Hands-free voice-note JSONL: {NOTE_JSONL}")
    dashboard.main()
