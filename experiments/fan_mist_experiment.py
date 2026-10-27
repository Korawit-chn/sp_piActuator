#!/usr/bin/env python3
"""fan_mist_experiment.py - step the fan through PWM 5%..100% against a saturated dome.

    python fan_mist_experiment.py

Runs on the backend's own machine, so it talks to localhost. There are no
arguments - everything about the experiment is a constant below.

One round, repeated until the fan has run at 100%:

    1. mist ON, wait until sensorID 1 reads >= TARGET_HUMIDITY
    2. mist OFF
    3. fan SET_SPEED at the current PWM, for FAN_SECONDS
    4. record the moment the fan Pi confirms it is running, and move the
       current PWM up by PWM_STEP
    5. wait for the run to finish

Everything goes through the dashboard API - the same calls the dashboard's own
buttons make - so the Pis stay in charge of the hardware and every command
shows up in ActuatorLog. Nothing here touches a GPIO pin.

THE PROGRESS FILE

One .txt next to this script, created on the first run, is both the result and
the resume point:

    current PWM: 15% (start 5%)
    Time Record
    5%: 2026-10-27 15:58:59
    10%: 2026-10-27 16:04:12

"current PWM" moves up the moment a time is recorded, so stopping the script
and starting it again carries on from that line. After 100% is recorded it
reads "current PWM: complete" and a further run does nothing. Delete or rename
the file to start a new experiment.

WHICH TIME IS RECORDED

The fan Pi reports back through /api/actuatorState right after it applies a
duty, and /api/actuators carries the server time of that report. That report
is the recorded time, not the moment this script sent the command - the Pi only
polls every few seconds, so the send time can be several seconds early. If no
confirmation arrives the send time is recorded instead, marked "(unconfirmed)".

SAFETY

The fan is always sent with a duration, so the fan Pi stops it on its own
even if this script dies mid-run. The mist is sent with the backend's longest
duration and re-sent while waiting, which only pushes the mist Pi's deadline
out - its relay is already ON, so nothing is pulsed. Ctrl-C, or any error,
sends OFF to both before exiting.

Humidity comes from GET /api/logs, which is only as fresh as the sensor's last
upload (sendSeconds in the dashboard's schedule, 60 s by default). A short
send period makes "reached the target" much less late.
"""

import sys
import time
from datetime import datetime
from pathlib import Path

import requests

BASE_DIR = Path(__file__).resolve().parent

BACKEND = "http://localhost:5000"
PROGRESS_FILE = BASE_DIR / "fan_experiment.txt"

# --- The experiment ---
TARGET_HUMIDITY = 100.0     # a DHT22 often tops out at 99.x - lower this if so
PWM_START = 5
PWM_STEP = 5
FAN_SECONDS = 30
# Give up, and turn everything off, if the target is not reached by then.
MAX_MIST_MINUTES = 30

# --- Which devices ---
SENSOR_ID = 1
LOCATION = "Inside Dome 1"
FAN_TYPE = "FAN"            # Type: in config_fan.txt
MIST_TYPE = "Mist Maker"    # Type: in config.txt

REQUEST_TIMEOUT = 5

# MAX_MIST_SECONDS in the backend's config.js. A longer ask is clamped to this
# anyway, so re-sending well inside it keeps the mister running without a gap.
MIST_RUN_SECONDS = 600
MIST_RESEND_SECONDS = 300

HUMIDITY_POLL_SECONDS = 10
# Faster than the humidity poll on purpose, and only for the few seconds after
# a fan command. The recorded time is the server's timestamp of the Pi's first
# report, and the Pi re-reports every 15 s, overwriting it - a poll slower than
# that could record the second report instead of the first.
CONFIRM_POLL_SECONDS = 2
# fan_control.py polls every 5 s and measures rpm for 1 s before it reports, so
# a confirmation normally lands inside ~7 s. This is the give-up point.
CONFIRM_TIMEOUT_SECONDS = 30

# How far back /api/logs is asked to look. Wide enough to cover one upload
# interval of a slow sensor, small enough to stay cheap at a 10 s poll.
LOG_WINDOW_HOURS = 0.25

TIME_FORMAT = "%Y-%m-%d %H:%M:%S"


def log(message):
    print(f"[{datetime.now().strftime(TIME_FORMAT)}] {message}", flush=True)


# ---------------------------------------------------------------------------
# Progress file
# ---------------------------------------------------------------------------

class Progress:
    """The .txt file: the current PWM and one recorded time per PWM."""

    def __init__(self, path):
        self.path = Path(path)
        self.start = PWM_START
        self.current = PWM_START      # None once 100% has been recorded
        self.records = []             # [(pwm, text)]

    def load(self):
        """Read an existing file, or leave the defaults for a fresh run."""
        if not self.path.exists():
            return

        for line in self.path.read_text(encoding="utf-8").splitlines():
            line = line.strip()

            if line.lower().startswith("current pwm:"):
                rest = line.split(":", 1)[1].strip()
                current, _, start = rest.partition("(start")
                current = current.strip()
                self.current = None if current == "complete" else _percent(current)

                if start:
                    self.start = _percent(start.rstrip(")").strip())

            elif "%:" in line:
                pwm, text = line.split("%:", 1)
                self.records.append((_percent(pwm), text.strip()))

        if self.current is None and not self.records:
            raise SystemExit(f"{self.path}: says complete but has no records")

    def record(self, pwm, text):
        """Write one time and move current up - in one write, so a crash
        cannot leave a recorded PWM that is still marked current."""
        self.records = [r for r in self.records if r[0] != pwm]
        self.records.append((pwm, text))

        nxt = pwm + PWM_STEP
        self.current = nxt if nxt <= 100 else None
        self.save()

    def save(self):
        current = "complete" if self.current is None else f"{_fmt(self.current)}%"
        lines = [f"current PWM: {current} (start {_fmt(self.start)}%)", "Time Record"]
        lines += [f"{_fmt(pwm)}%: {text}" for pwm, text in self.records]

        # Atomic, same as the Pis' state files: a half-written progress file
        # would resume from the wrong PWM.
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text("\n".join(lines) + "\n", encoding="utf-8")
        tmp.replace(self.path)


def _percent(text):
    return float(text.strip().rstrip("%"))


def _fmt(pwm):
    return f"{pwm:g}"


# ---------------------------------------------------------------------------
# Dashboard API
# ---------------------------------------------------------------------------

class Api:
    def __init__(self, base_url):
        self.base = base_url.rstrip("/")
        self.session = requests.Session()

    def _get(self, path, **params):
        r = self.session.get(f"{self.base}{path}", params=params, timeout=REQUEST_TIMEOUT)
        r.raise_for_status()
        return r.json()

    def command(self, actuator_id, action, duty=None, seconds=None):
        body = {"actuatorID": actuator_id, "action": action,
                "triggerSource": "AUTOMATION"}
        if duty is not None:
            body["pwmDutyPercent"] = duty
        if seconds is not None:
            body["durationSeconds"] = int(seconds)

        r = self.session.post(f"{self.base}/api/actuatorCommand", json=body,
                              timeout=REQUEST_TIMEOUT)
        r.raise_for_status()
        return r.json()

    def actuators(self):
        return self._get("/api/actuators")

    def actuator(self, actuator_id):
        data = self.actuators()
        for a in data["actuators"]:
            if a["actuatorID"] == actuator_id:
                return a, data["serverEpochMs"]
        raise SystemExit(f"actuator {actuator_id} has disappeared from /api/actuators")

    def find_actuator(self, type_name):
        matches = [a for a in self.actuators()["actuators"]
                   if (a.get("typeName") or "").lower() == type_name.lower()
                   and (a.get("locationName") or "").lower() == LOCATION.lower()]

        if len(matches) != 1:
            raise SystemExit(
                f"expected one {type_name!r} actuator at {LOCATION!r}, found "
                f"{len(matches)} - check FAN_TYPE / MIST_TYPE / LOCATION")

        return matches[0]["actuatorID"]

    def latest_humidity(self):
        """Newest (datetime, humidity) from SENSOR_ID, or None.

        datetime is compared only against other values from this same endpoint,
        never against this machine's clock, so a timezone difference between
        the PC and the database cannot make a stale reading look new.
        """
        rows = self._get("/api/logs", hours=LOG_WINDOW_HOURS)

        for row in reversed(rows):
            if row.get("sensorID") == SENSOR_ID and row.get("humidity") is not None:
                return row["datetime"], float(row["humidity"])

        return None


# ---------------------------------------------------------------------------
# One round
# ---------------------------------------------------------------------------

def saturate(api, mist_id):
    """Mist until the sensor reads >= target, then stop the mist.

    Only a reading newer than the newest one at the moment the mist started
    counts, so humidity left over from the last round cannot end this one
    before any misting has happened.
    """
    before = api.latest_humidity()
    baseline = before[0] if before else None

    log(f"mist ON - waiting for humidity >= {TARGET_HUMIDITY:g}%"
        + (f" (now {before[1]:.1f}%)" if before else ""))

    started = time.monotonic()
    last_send = None
    last_shown = None

    while True:
        now = time.monotonic()

        if last_send is None or now - last_send >= MIST_RESEND_SECONDS:
            api.command(mist_id, "ON", seconds=MIST_RUN_SECONDS)
            last_send = now

        if now - started > MAX_MIST_MINUTES * 60:
            raise RuntimeError(
                f"humidity did not reach {TARGET_HUMIDITY:g}% within "
                f"{MAX_MIST_MINUTES:g} min - lower TARGET_HUMIDITY or raise "
                f"MAX_MIST_MINUTES")

        reading = api.latest_humidity()

        if reading and reading[0] != last_shown and (baseline is None or reading[0] > baseline):
            last_shown = reading[0]
            log(f"humidity {reading[1]:.1f}%")

            if reading[1] >= TARGET_HUMIDITY:
                break

        time.sleep(HUMIDITY_POLL_SECONDS)

    api.command(mist_id, "OFF")
    log(f"mist OFF after {time.monotonic() - started:.0f}s")


def run_fan(api, fan_id, pwm):
    """Start the fan at pwm and return the time it started, as text.

    The 30 s count is the fan Pi's own timer, armed when it applies the duty -
    the same moment it reports, which is the time recorded here.
    """
    _, sent_ms = api.actuator(fan_id)
    api.command(fan_id, "SET_SPEED", duty=pwm, seconds=FAN_SECONDS)
    sent_at = datetime.now()
    log(f"fan SET_SPEED {_fmt(pwm)}% for {FAN_SECONDS:g}s - waiting for the Pi")

    deadline = time.monotonic() + CONFIRM_TIMEOUT_SECONDS

    while time.monotonic() < deadline:
        a, _ = api.actuator(fan_id)
        duty = a.get("lastDutyPercent")
        reported_ms = a.get("statusUpdatedAtMs") or 0

        # The command itself is an ActuatorLog row carrying the same duty, and
        # it bumps status and statusUpdatedAt too, so neither proves anything
        # on its own. Only the Pi's telemetry row has a pulseCount - that is
        # what tells the report apart from the request.
        if (a.get("status") == "ON" and a.get("lastPulseCount") is not None
                and duty is not None and abs(float(duty) - pwm) < 0.01
                and reported_ms >= sent_ms):
            started = datetime.fromtimestamp(reported_ms / 1000)
            log(f"fan confirmed at {_fmt(pwm)}%")
            return started.strftime(TIME_FORMAT)

        time.sleep(CONFIRM_POLL_SECONDS)

    log("fan did not confirm - recording the time the command was sent")
    return sent_at.strftime(TIME_FORMAT) + " (unconfirmed)"


def wait_fan_done(api, fan_id, started):
    """Sleep out the run, then wait for the Pi to report the fan stopped.

    Waiting for the OFF report as well as the clock keeps the next round's
    misting from overlapping a fan the Pi has not stopped yet.
    """
    time.sleep(max(0.0, started + FAN_SECONDS - time.monotonic()))

    deadline = time.monotonic() + CONFIRM_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        a, _ = api.actuator(fan_id)
        if a.get("status") != "ON":
            log("fan stopped")
            return
        time.sleep(CONFIRM_POLL_SECONDS)

    log("fan still reports ON - sending OFF")
    api.command(fan_id, "OFF")


def stop_all(api, ids):
    for actuator_id in ids:
        try:
            api.command(actuator_id, "OFF")
        except requests.RequestException as e:
            log(f"could not send OFF to actuator {actuator_id}: {e}")


# ---------------------------------------------------------------------------

def main():
    api = Api(BACKEND)

    progress = Progress(PROGRESS_FILE)
    progress.load()

    if progress.current is None:
        log(f"{progress.path} is already complete - nothing to do")
        return 0

    progress.save()
    log(f"progress: {progress.path}")
    log(f"starting at {_fmt(progress.current)}% (start {_fmt(progress.start)}%, "
        f"step {_fmt(PWM_STEP)}%)")

    fan_id = api.find_actuator(FAN_TYPE)
    mist_id = api.find_actuator(MIST_TYPE)
    log(f"fan actuatorID {fan_id}, mist actuatorID {mist_id}, sensorID {SENSOR_ID}")

    try:
        while progress.current is not None:
            pwm = progress.current
            log(f"=== round {_fmt(pwm)}% ===")

            saturate(api, mist_id)

            started = time.monotonic()
            text = run_fan(api, fan_id, pwm)
            progress.record(pwm, text)
            log(f"recorded {_fmt(pwm)}%: {text}")

            wait_fan_done(api, fan_id, started)

        log("experiment complete")
        return 0

    except KeyboardInterrupt:
        log("interrupted - stopping mist and fan")
        return 130

    except (requests.RequestException, RuntimeError) as e:
        log(f"stopping: {e}")
        return 1

    finally:
        stop_all(api, (mist_id, fan_id))


if __name__ == "__main__":
    sys.exit(main())
