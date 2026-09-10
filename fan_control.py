#!/usr/bin/env python3
"""fan_control.py - PWM fan speed control and tach reading.

    python3 fan_control.py            # obey the dashboard (systemd runs this)
    python3 fan_control.py 50         # bench: hold 50% and print rpm, no backend
    python3 fan_control.py 50 300     # bench: hold 50% for 300s, then stop

Registers itself against the shared device UUID, then polls the dashboard for a
command and reports back what the hardware is actually doing:

    OFF        -> duty 0
    ON         -> OnDuty from config_fan.txt
    SET_SPEED  -> the explicit duty the command carries, the only case where an
                  arbitrary value is applied

A RUN ENDS ON TIME, OR WHEN IT IS TOLD TO

A command that spins the fan up may also say for how long, exactly like the
mist maker: send a duty AND a duration and this file stops the fan when the
time is up, clamped to MaxRun from config_fan.txt.

A command that names NO duration has no end time. It runs until an OFF, or a
SET_SPEED of 0, reaches it. That is the deliberate second form and it costs the
guarantee the paragraph below describes: with no deadline to fall back on, a
fan started this way does NOT stop by itself if the dashboard disappears
mid-run. It is for a run someone is watching. A duration is the form that
survives an outage, and it is still what the dashboard sends unless the
duration box is left empty.

Only a MISSING duration means that. A present-but-unusable one - a zero, a
negative, a value that will not parse - is a malformed request rather than a
request for no timer, and still gets MaxRun.

The timer, when there is one, is checked BEFORE the network on every cycle,
which is the whole point of keeping it here rather than having the backend send
a later OFF: if the dashboard disappears mid-run, the fan still stops. Setting duty 0 is also
the one thing that is always safe to re-apply, so a stop cannot be lost the way
a latching relay's could.

Telemetry (duty applied, pulse count, measured rpm) goes to /api/actuatorStatus,
which writes an ActuatorLog row and nothing else. It is NOT a heartbeat - the
device agent reports whether this Pi is alive, once per box.

Requires pigpio and its daemon:  sudo apt install pigpio && sudo pigpiod

WHAT THIS FILE NO LONGER CONTAINS

Backend discovery, the device-UUID reader and the config parser. All three were
written out here a fourth time - the file's own docstring used to admit it
"mirrors sensorVPD.client.BackendClient.discover_backend ... just inlined here
rather than pulled in as a package import". They come from pi_common now, so
this file cannot disagree with the sensors and the mister about what a config
file is or which backend to talk to.

It also read fan_config.json - the only JSON config in the project - and
imported a generate_uuid module that does not exist, so it could not start at
all. Both are gone: same `key: value` config.txt as everything else, same
device_uuid.txt.

WHY NO BELIEVED-STATE FILE

Unlike the mist relay, a PWM fan is level-based and has readback. Setting a duty
is idempotent, and the tach says what actually happened, so there is no drift to
track and nothing to converge on restart: startup drives the pin to 0 and that
is the truth.

fan_state.json exists anyway, and holds ONE thing - the actionID last applied.
That is dedupe, not believed state. Without it, anything that clears the
in-memory copy - a restart, or a reconnect that re-registers - makes the
still-current command look new, re-applies it and RE-ARMS the run timer. On a
flaky link that pushes the deadline out on every blip, and a fan asked for ten
seconds never stops.

It is deliberately not used to resume a run. Startup is always duty 0, so an
interrupted run is abandoned rather than silently restarted; the Pi reports 0
on its next status post, so the dashboard shows a stopped fan instead of
claiming one that is not running.
"""

import json
import os
import signal
import sys
import threading
import time
from pathlib import Path

import pigpio
import requests

from pi_common.config import load_config_data, to_number
from pi_common.discovery import find_backend
from pi_common.identity import get_device_uuid

# Anchored to this folder, not the cwd, so launching from somewhere else cannot
# quietly create a second identity.
BASE_DIR = Path(__file__).resolve().parent

# config_fan.txt, not config.txt: the mister already owns that name, and the Pi
# is one flat directory, so a box driving both would otherwise have them read
# each other's settings.
CONFIG_FILE = Path(os.environ.get("FAN_CONFIG", BASE_DIR / "config_fan.txt"))

# One UUID and one backend list per Pi, shared with every other client here.
UUID_FILE = Path(os.environ.get("SENSOR_UUID_FILE", BASE_DIR / "device_uuid.txt"))
NETWORK_LIST = Path(os.environ.get("SP_NETWORK_LIST", BASE_DIR / "networkList.txt"))

# The applied actionID, across restarts. Named like mist_state.json and written
# the same way, but it carries no hardware state - see the docstring.
STATE_FILE = BASE_DIR / "fan_state.json"

BACKEND_PORT = int(os.environ.get("SP_BACKEND_PORT", 5000))
REQUEST_TIMEOUT = 5

DEFAULT_PWM_PIN = 18          # hardware PWM channel on a Pi
DEFAULT_TACH_PIN = 24
DEFAULT_PWM_FREQ_HZ = 25000   # 25 kHz, the Intel 4-wire fan spec
DEFAULT_PULSES_PER_REV = 2
DEFAULT_ON_DUTY = 100

# Longest a single run may last, and the fallback for a command that named no
# duration. Twin of MAX_FAN_SECONDS in the backend's config.js - DELIBERATELY
# duplicated, exactly as the mist maker's MaxRun is, because this is the copy
# that still applies when the backend is unreachable.
DEFAULT_MAX_RUN_SECONDS = 3600

DEFAULT_POLL_SECONDS = 5
DEFAULT_REPORT_SECONDS = 15
DEFAULT_RPM_WINDOW = 1.0

stop_event = threading.Event()


def handle_stop(signum, frame):
    print("[fan] stopping...")
    stop_event.set()


# ---------------------------------------------------------------------------
# Applied-command state
# ---------------------------------------------------------------------------

# The last actionID this Pi actually applied. In memory it would be lost by
# every restart and every re-registration; on disk it survives both, which is
# the whole point - see WHY NO BELIEVED-STATE FILE.
STATE = {"lastActionID": None}


def load_state():
    """Read the applied actionID. Absent or unreadable means None.

    None is safe in the direction that matters: the next command is treated as
    new and applied. The failure it cannot cause is a fan left spinning, since
    startup sets duty 0 regardless of what is in here.
    """
    try:
        data = json.loads(STATE_FILE.read_text())
    except (OSError, ValueError):
        STATE["lastActionID"] = None
        return

    STATE["lastActionID"] = data.get("lastActionID")


def save_state():
    """Write it atomically - a truncated file on the next start reads as None."""
    try:
        tmp = STATE_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(STATE))
        os.replace(tmp, STATE_FILE)
    except OSError as e:
        # Not fatal. The run still ends on time from the in-memory deadline;
        # only the restart case loses its dedupe.
        print(f"[fan] cannot write {STATE_FILE.name}: {e}")


# ---------------------------------------------------------------------------
# Config - same format and same parser as the sensors and the mister
# ---------------------------------------------------------------------------

def read_config():
    try:
        data = load_config_data(CONFIG_FILE)
    except OSError as e:
        raise SystemExit(f"[fan] cannot read {CONFIG_FILE}: {e}")

    actuator_type = data.get("Type") or data.get("actuatorType")
    location = data.get("Location") or data.get("locationName")

    # /api/registerActuator rejects a payload without these, and an actuator
    # that never registers just polls nothing forever. Failing here names the
    # file instead of leaving a 400 to be traced back from the dashboard.
    if not actuator_type or not location:
        raise SystemExit(
            f"[fan] {CONFIG_FILE}: 'Type' and 'Location' are required "
            f"(got Type={actuator_type!r}, Location={location!r})"
        )

    return {
        "deviceUUID": get_device_uuid(UUID_FILE),
        "actuatorType": actuator_type,
        "locationName": location,
        "actuatorName": data.get("Name") or data.get("actuatorName"),
        "description": data.get("description") or data.get("Description"),
        "pwmPin": to_number(data.get("PwmPin"), DEFAULT_PWM_PIN),
        "tachPin": to_number(data.get("TachPin"), DEFAULT_TACH_PIN),
        "pwmFrequencyHz": to_number(data.get("PwmHz"), DEFAULT_PWM_FREQ_HZ),
        "pulsesPerRev": to_number(data.get("PulsesPerRev"), DEFAULT_PULSES_PER_REV),
        "onDutyPercent": to_number(data.get("OnDuty"), DEFAULT_ON_DUTY),
        "maxRunSeconds": to_number(data.get("MaxRun"), DEFAULT_MAX_RUN_SECONDS),
        "pollSeconds": to_number(data.get("Poll"), DEFAULT_POLL_SECONDS),
        "reportSeconds": to_number(data.get("Report"), DEFAULT_REPORT_SECONDS),
        "rpmWindowSeconds": to_number(data.get("RpmWindow"), DEFAULT_RPM_WINDOW),
    }


# ---------------------------------------------------------------------------
# Hardware
# ---------------------------------------------------------------------------

class Fan:
    """The PWM pin and the tach pin. The only part of this file about a fan."""

    def __init__(self, config):
        self.pwm_pin = config["pwmPin"]
        self.tach_pin = config["tachPin"]
        self.freq_hz = config["pwmFrequencyHz"]
        self.pulses_per_rev = config["pulsesPerRev"]

        self.pi = pigpio.pi()

        if not self.pi.connected:
            raise SystemExit(
                "[fan] cannot reach pigpiod - start it with: sudo pigpiod"
            )

        self.pi.set_mode(self.tach_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.tach_pin, pigpio.PUD_UP)

        self._pulses = 0
        self._lock = threading.Lock()
        self.pi.callback(self.tach_pin, pigpio.FALLING_EDGE, self._on_pulse)

        self.set_duty(0)

    def _on_pulse(self, gpio, level, tick):
        with self._lock:
            self._pulses += 1

    def set_duty(self, duty_percent):
        """Clamped to 0-100. Returns what was actually applied."""
        duty = max(0.0, min(100.0, float(duty_percent)))
        # pigpio takes duty in millionths.
        self.pi.hardware_PWM(self.pwm_pin, self.freq_hz, int(duty * 10000))
        return duty

    def pulse_count(self):
        with self._lock:
            return self._pulses

    def measure_rpm(self, window_seconds):
        """Zero the tach counter, wait, convert pulses to rpm.

        Waits on the stop_event rather than sleeping, so SIGTERM during the
        measurement window stops the process now rather than up to a second
        later. A truncated window would misreport, so an interrupted
        measurement returns None instead of a number.
        """
        with self._lock:
            self._pulses = 0

        if stop_event.wait(window_seconds):
            return None

        with self._lock:
            count = self._pulses

        return (count / self.pulses_per_rev) * (60.0 / window_seconds)

    def close(self):
        try:
            self.pi.hardware_PWM(self.pwm_pin, self.freq_hz, 0)
            self.pi.stop()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Backend
# ---------------------------------------------------------------------------

LINK = {"base_url": None, "actuatorID": None}


def ensure_backend(session):
    """Keep the current backend while it answers; rescan only when it stops."""
    previous = LINK["base_url"]

    if LINK["base_url"] is not None:
        try:
            r = session.get(f"{LINK['base_url']}/api/time", timeout=3)
            if r.status_code == 200:
                return LINK["base_url"]
        except requests.RequestException:
            pass

        print(f"[fan] lost backend {LINK['base_url']}, rescanning")
        LINK["base_url"] = None

    LINK["base_url"] = find_backend(str(NETWORK_LIST), port=BACKEND_PORT)

    if LINK["base_url"]:
        print("[fan] backend:", LINK["base_url"])

        # Only a DIFFERENT backend invalidates the identity. actuatorIDs and
        # actionIDs are one database's numbering, so a rescan that lands
        # somewhere else has to re-register - but a blip and a reconnect to the
        # SAME PC must not, or the re-registration clears the applied actionID,
        # the still-current command reads as new, and the run timer is re-armed.
        if previous is not None and LINK["base_url"] != previous:
            print("[fan] different backend - re-registering")
            LINK["actuatorID"] = None
            STATE["lastActionID"] = None
            save_state()

    return LINK["base_url"]


def register(session, config):
    r = session.post(
        f"{LINK['base_url']}/api/registerActuator",
        json={
            "deviceUUID": config["deviceUUID"],
            "actuatorType": config["actuatorType"],
            "locationName": config["locationName"],
            "actuatorName": config["actuatorName"],
            "description": config["description"],
        },
        timeout=REQUEST_TIMEOUT,
    )
    r.raise_for_status()
    return r.json().get("actuatorID")


def fetch_command(session):
    """Latest command as (actionID, action, duty, duration_seconds).

    pwmDutyPercent comes back as a STRING - mysql2 returns DECIMAL that way -
    so it is cast once here rather than at every use. durationSeconds is a
    plain INT column and needs no such care, but it is normalised alongside so
    a malformed value cannot reach the timer arithmetic.
    """
    r = session.get(
        f"{LINK['base_url']}/api/actuatorCommand",
        params={"actuatorID": LINK["actuatorID"]},
        timeout=REQUEST_TIMEOUT,
    )
    r.raise_for_status()
    data = r.json()

    duty = data.get("pwmDutyPercent")

    try:
        duty = None if duty is None else float(duty)
    except (TypeError, ValueError):
        duty = None

    # None means the command named NO duration, which run_until() reads as "no
    # deadline". A value that will not parse must NOT collapse to the same
    # thing - that would turn a malformed request into a fan running forever -
    # so it becomes -1, which run_until() treats like any other non-positive
    # duration and answers with MaxRun.
    raw_duration = data.get("durationSeconds")

    if raw_duration is None:
        duration = None
    else:
        try:
            duration = float(raw_duration)
        except (TypeError, ValueError):
            duration = -1.0

    return data.get("actionID"), data.get("action"), duty, duration


def report(session, duty, pulses, rpm):
    """Telemetry. Writes one ActuatorLog row; liveness is the agent's job."""
    session.post(
        f"{LINK['base_url']}/api/actuatorStatus",
        json={
            "actuatorID": LINK["actuatorID"],
            "pwmDutyPercent": duty,
            "pulseCount": pulses,
            "rpm": None if rpm is None else round(rpm),
        },
        timeout=REQUEST_TIMEOUT,
    )


def duty_for(action, duty, on_duty):
    """ON and OFF are fixed points, so they never inherit whatever duty a
    previous SET_SPEED left behind. Anything unrecognised fails safe to 0."""
    if action == "ON":
        return on_duty
    if action == "SET_SPEED":
        return duty if duty is not None else on_duty
    return 0


def run_until(duty, requested, max_run):
    """When a run at `duty` should end, as a monotonic deadline, or None.

    None means there is nothing to time, and there are now two ways to get it.

    Duty 0: a stopped fan is already in the state the timer exists to reach, so
    arming one would just re-apply 0 to a fan that is already at 0.

    `requested` of None: the command named no duration at all, which is a
    deliberate request to run until an OFF arrives. See the header - it is the
    form that does NOT survive the dashboard going away mid-run, and it is
    chosen rather than fallen into.

    Falling into it is what the second branch prevents. A zero, a negative, or
    a value fetch_command() could not parse (which it passes on as -1) is a
    malformed request, not a request for no timer, so it still gets max_run.
    """
    if duty <= 0:
        return None

    if requested is None:
        return None

    seconds = requested if requested > 0 else max_run

    return time.monotonic() + min(seconds, max_run)


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------

def bench(duty_percent, seconds=None):
    """Hold one duty and print rpm. No backend, no registration.

    With a duration it runs the same deadline the service does, through the
    same run_until() - a bench run that ignored MaxRun would prove nothing
    about the deployed behaviour. Without one it holds until Ctrl-C, which is
    the one place a fan may spin with no timer: someone is standing there.
    """
    config = read_config()
    fan = Fan(config)

    applied = fan.set_duty(duty_percent)
    off_at = (None if seconds is None
              else run_until(applied, seconds, config["maxRunSeconds"]))

    print(f"[fan] bench: holding {applied}%"
          + (" - Ctrl-C to stop" if off_at is None
             else f" for {off_at - time.monotonic():.0f}s"))

    try:
        while not stop_event.is_set():
            if off_at is not None and time.monotonic() >= off_at:
                print("[fan] bench run finished")
                break

            rpm = fan.measure_rpm(config["rpmWindowSeconds"])
            if rpm is not None:
                print(f"[fan] {applied:.0f}%  {rpm:.0f} rpm")
    finally:
        fan.close()


def serve():
    config = read_config()

    print(f"[fan] {config['actuatorType']} @ {config['locationName']}, "
          f"pwm pin {config['pwmPin']}, tach pin {config['tachPin']}, "
          f"poll {config['pollSeconds']}s, max run {config['maxRunSeconds']}s")

    fan = Fan(config)
    session = requests.Session()

    # Fan() has already set duty 0, so the hardware is known-off here whatever
    # happened last time. What is restored is only the applied actionID, which
    # is what stops the command still sitting on the dashboard from being
    # re-applied and spinning the fan straight back up.
    load_state()

    applied_duty = 0.0
    last_report = 0.0
    off_at = None

    try:
        while not stop_event.is_set():
            # Measured every cycle whether or not the backend is reachable: the
            # tach is local, and an unreachable dashboard is no reason to stop
            # knowing what the fan is doing.
            rpm = fan.measure_rpm(config["rpmWindowSeconds"])

            if stop_event.is_set():
                break

            # Before the network, always. The run has to end on time whether or
            # not the dashboard is reachable - that is the entire reason the
            # deadline is kept here instead of the backend sending a later OFF.
            if off_at is not None and time.monotonic() >= off_at:
                applied_duty = fan.set_duty(0)
                off_at = None
                print("[fan] run finished - duty 0%")
                # Report it now rather than at the next reportSeconds boundary,
                # so the panel stops showing a speed the fan is no longer at.
                last_report = 0.0

            try:
                if ensure_backend(session) is None:
                    print("[fan] backend unavailable")
                    stop_event.wait(config["pollSeconds"])
                    continue

                if LINK["actuatorID"] is None:
                    LINK["actuatorID"] = register(session, config)
                    print("[fan] registered actuatorID:", LINK["actuatorID"])

                action_id, action, duty, duration = fetch_command(session)

                # Applied once per command, like the mister: re-applying a
                # level-based duty is harmless, but tracking the id keeps the
                # log honest about when the fan actually changed.
                #
                # Applying it is also what arms the timer, so a re-served
                # command cannot keep pushing the deadline out - which is the
                # same reason the mister tracks lastActionID. It is kept on
                # disk for the same reason too: a restart that forgot it would
                # re-apply the command and re-arm the deadline from zero.
                if action_id != STATE["lastActionID"]:
                    STATE["lastActionID"] = action_id
                    save_state()
                    applied_duty = fan.set_duty(
                        duty_for(action, duty, config["onDutyPercent"])
                    )
                    off_at = run_until(applied_duty, duration,
                                       config["maxRunSeconds"])

                    # An untimed run and a stop both leave off_at None, so say
                    # which this is. "duty 60%" with nothing after it would
                    # otherwise be the only trace of a fan that intends to keep
                    # spinning until something stops it.
                    if off_at is not None:
                        for_text = f" for {off_at - time.monotonic():.0f}s"
                    elif applied_duty > 0:
                        for_text = " until stopped - no timer"
                    else:
                        for_text = ""
                    print(f"[fan] {action} -> duty {applied_duty:.0f}%{for_text}")

                now = time.monotonic()

                if now - last_report >= config["reportSeconds"]:
                    report(session, applied_duty, fan.pulse_count(), rpm)
                    last_report = now

            except requests.RequestException as e:
                # Network trouble only. Anything else is a real bug: let it
                # crash and let systemd restart us clean.
                print("[fan] backend error:", e)
                LINK["base_url"] = None

            stop_event.wait(config["pollSeconds"])

    finally:
        # Always leave the fan stopped. A crash-restart loop ends with it off
        # rather than spinning unattended.
        fan.close()
        print("[fan] stopped")


USAGE = """usage: fan_control.py [duty [seconds]]

  (no argument)   poll the dashboard and obey it (this is what systemd runs)
  <duty>          bench: hold duty% and print rpm, without touching the backend
  <duty> <secs>   bench: the same, stopping after secs (clamped to MaxRun)
"""


def _positive_number(text):
    """A bench argument, or None if it is not a number at all."""
    try:
        value = float(text)
    except ValueError:
        return None

    return value if value > 0 else None


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, handle_stop)
    signal.signal(signal.SIGINT, handle_stop)

    if len(sys.argv) == 1:
        serve()
        sys.exit(0)

    if len(sys.argv) > 3:
        sys.exit(USAGE)

    duty = _positive_number(sys.argv[1]) if sys.argv[1] != "0" else 0.0

    if duty is None:
        sys.exit(USAGE)

    seconds = None

    if len(sys.argv) == 3:
        seconds = _positive_number(sys.argv[2])

        if seconds is None:
            sys.exit(f"[fan] run length must be a positive number of "
                     f"seconds, got {sys.argv[2]!r}\n\n" + USAGE)

    bench(duty, seconds)
