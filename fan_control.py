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

EVERY RUN ENDS

A command that spins the fan up also says for how long, exactly like the mist
maker: the dashboard sends a duty AND a duration, and this file stops the fan
when the time is up. A command that arrives without one gets MaxRun from
config_fan.txt, so there is no run-forever mode to fall into by omission.

The timer is checked BEFORE the network on every cycle, which is the whole
point of keeping it here rather than having the backend send a later OFF: if
the dashboard disappears mid-run, the fan still stops. Setting duty 0 is also
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
track and nothing to converge on restart.
"""

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
    if LINK["base_url"] is not None:
        try:
            r = session.get(f"{LINK['base_url']}/api/time", timeout=3)
            if r.status_code == 200:
                return LINK["base_url"]
        except requests.RequestException:
            pass

        print(f"[fan] lost backend {LINK['base_url']}, rescanning")
        LINK["base_url"] = None
        LINK["actuatorID"] = None

    LINK["base_url"] = find_backend(str(NETWORK_LIST), port=BACKEND_PORT)

    if LINK["base_url"]:
        print("[fan] backend:", LINK["base_url"])

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

    duration = data.get("durationSeconds")

    try:
        duration = None if duration is None else float(duration)
    except (TypeError, ValueError):
        duration = None

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

    None means there is nothing to time. That is duty 0 and only duty 0: a
    stopped fan is already in the state the timer exists to reach, so arming
    one would just re-apply 0 to a fan that is already at 0.

    A command with no duration is NOT a run-forever request - it gets max_run.
    The dashboard always sends one, so this covers a climate rule or a curl
    that did not, and it is the reason there is no code path here that leaves a
    spinning fan with no deadline.
    """
    if duty <= 0:
        return None

    seconds = max_run if requested is None or requested <= 0 else requested

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

    applied_duty = 0.0
    last_action_id = None
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
                    last_action_id = None

                action_id, action, duty, duration = fetch_command(session)

                # Applied once per command, like the mister: re-applying a
                # level-based duty is harmless, but tracking the id keeps the
                # log honest about when the fan actually changed.
                #
                # Applying it is also what arms the timer, so a re-served
                # command cannot keep pushing the deadline out - which is the
                # same reason the mister tracks lastActionID.
                if action_id != last_action_id:
                    last_action_id = action_id
                    applied_duty = fan.set_duty(
                        duty_for(action, duty, config["onDutyPercent"])
                    )
                    off_at = run_until(applied_duty, duration,
                                       config["maxRunSeconds"])

                    for_text = ("" if off_at is None
                                else f" for {off_at - time.monotonic():.0f}s")
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
