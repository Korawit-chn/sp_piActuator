"""relay_control.py — mist maker control, by hand or from the dashboard.

    python3 relay_control.py 60      # mist for 60 seconds, then stop
    python3 relay_control.py serve   # obey the dashboard (systemd runs this)
    python3 relay_control.py flip    # one raw pulse, see "Drift" below

Built to match the sensor Pi clients: same `config.txt` format, the same
`device_uuid.txt`, the same `networkList.txt`, the same register-then-poll
shape. What it deliberately does NOT have, because a command is not data:
no offline cache, no clock sync, no tick scheduler, no background thread.

Every trigger is still sent by launching mist_trigger.py as a separate
process. A fresh process start-to-exit is what reliably produces one trigger
event on this board, and that has not changed. The pin comes from `GPIO:` in
config.txt and is passed to it as an argument.

THE BOARD TOGGLES
    It is edge-triggered and latching: no "set ON", no "set OFF", just one
    input that flips it to whatever it was not. There is no readback either -
    nothing here can ask the board what state it is in.

    The dashboard, meanwhile, is level-based: GET /api/actuatorCommand re-serves
    the latest command on every poll. Acting on that directly would pulse the
    relay every few seconds and the mister would strobe.

    So we track a believed state on disk and fire only on a transition. That
    is set_state(), and it is the ONLY thing allowed to call fire() in normal
    operation. Add a second caller and the belief silently drifts away from
    the hardware.

DRIFT
    A missed pulse, a power glitch, someone pressing the button on the board -
    the belief can end up wrong and nothing in software can detect it. The
    dashboard then confidently shows the opposite of what the mister is doing.

    With two states, a disagreement is always exactly one toggle out, so one
    pulse fixes it: stop the service, run `relay_control.py flip`, start it
    again. The relay ends up matching what the dashboard already said. Rare
    enough that a bench command beats building UI for it.

LET IT CRASH
    systemd restarts this with Restart=always, so there is no retry ladder and
    no catch-all here. Only network errors are swallowed; anything else should
    kill the process and get restarted clean. One consequence, and it is the
    right one: a crash mid-run leaves the mister OFF and does not resume it.
    Someone presses Start again.
"""

import json
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

import requests

# Anchored to this folder, not the cwd, so launching from somewhere else
# cannot quietly create a second identity or a second state file.
BASE_DIR = Path(__file__).resolve().parent

TRIGGER = BASE_DIR / "mist_trigger.py"
STATE_FILE = BASE_DIR / "mist_state.json"
CONFIG_FILE = Path(os.environ.get("MIST_CONFIG", BASE_DIR / "config.txt"))
NETWORK_LIST = Path(os.environ.get("MIST_NETWORK_LIST", BASE_DIR / "networkList.txt"))

# Same filename and same meaning as the sensors: one UUID per physical Pi.
# On a Pi that also runs sensors, symlink this to "PI sensor/device_uuid.txt"
# BEFORE the first start - afterwards is too late, a second UUID has already
# been minted and registered.
UUID_FILE = Path(os.environ.get("SENSOR_UUID_FILE", BASE_DIR / "device_uuid.txt"))

USAGE = """usage: relay_control.py <mode>

  serve    poll the dashboard and obey it (this is what systemd runs)
  <n>      mist for n seconds, then stop
  flip     send one raw pulse, to bring a drifted relay back into
           agreement with what the dashboard shows

There is no default mode - see the note at the bottom of this file."""

BACKEND_PORT = 5000
DEFAULT_POLL_SECONDS = 5
DEFAULT_MAX_RUN_SECONDS = 600
DEFAULT_GPIO = "D17"
HEARTBEAT_SECONDS = 60

# Which pin mist_trigger.py should pulse. Read from `GPIO:` in config.txt at
# startup, for every mode - a bench run has to hit the same pin the service
# does, or testing proves nothing about the deployed behaviour.
GPIO_PIN = DEFAULT_GPIO

# What we last drove the board to, mirrored to STATE_FILE. lastActionID is what
# makes polling a level endpoint safe: a command is applied once and never
# re-applied, so our own auto-off is not undone on the next cycle by the stale
# ON still sitting at the top of the log.
STATE = {"believed": "OFF", "lastActionID": None}

# Held at module level purely so the shutdown handler can file a last report.
LINK = {"base_url": None, "actuatorID": None}


# ---------------------------------------------------------------------------
# The relay
# ---------------------------------------------------------------------------

def fire() -> None:
    """Send exactly one trigger to the relay board."""
    subprocess.run([sys.executable, str(TRIGGER), GPIO_PIN], check=True, timeout=30)


def load_state() -> None:
    """Read the believed state, defaulting to OFF."""
    try:
        data = json.loads(STATE_FILE.read_text())
    except (OSError, ValueError):
        # No state file, or an unreadable one. Reset rather than keeping
        # whatever happens to be in memory - "defaulting to OFF" has to be
        # true on every path, not just the first call.
        #
        # OFF is the only assumption that cannot itself turn the mister on.
        # But the board keeps its own state across a reboot, so if it really
        # is running, nothing here will notice. That is what `flip` is for,
        # and why this prints rather than failing quietly.
        STATE["believed"] = "OFF"
        STATE["lastActionID"] = None
        print(f"[mist] no usable {STATE_FILE.name} - assuming the relay is OFF, "
              f"check the hardware")
        return

    STATE["believed"] = data.get("believed") if data.get("believed") in ("ON", "OFF") else "OFF"
    STATE["lastActionID"] = data.get("lastActionID")


def save_state() -> None:
    """Persist atomically. A half-written state file is worse than none,
    because it would be trusted."""
    tmp = STATE_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(STATE))
    os.replace(tmp, STATE_FILE)


def set_state(target: str) -> bool:
    """Drive the board to `target`, firing at most once. Returns True if it
    actually pulsed.

    Persisting happens BEFORE the caller reports anything: report first and
    crash, and the dashboard would claim ON while the file says OFF.
    """
    if STATE["believed"] == target:
        return False

    fire()
    STATE["believed"] = target
    save_state()
    print(f"[mist] relay -> {target}")
    return True


def run_for(seconds: float) -> None:
    """Start the loop, wait, stop it.

    Goes through set_state() rather than fire() so a bench run leaves the state
    file correct and switching back to serve mode does not inherit a stale
    belief. The try/finally still matters for the same reason it always did:
    without it a Ctrl-C during the wait would skip the stop and leave the board
    looping with nothing tracking it.
    """
    set_state("ON")
    print(f"started — running for {seconds:.0f}s")
    try:
        time.sleep(seconds)
    finally:
        set_state("OFF")
        print("stopped")


def flip() -> None:
    """One raw pulse, leaving the believed state alone.

    For when the belief and the hardware have come apart. With only two states
    any disagreement is exactly one toggle, so a single pulse is enough - and
    it is the HARDWARE that moves, because the belief is what the dashboard is
    already showing.

    Deliberately does not touch the belief. Pulsing and inverting would move
    both, leaving them exactly as far apart as they started:

        drifted        believed OFF, actually ON
        after fire()   believed OFF, actually OFF   <- already correct
        after invert   believed ON,  actually OFF   <- broken again

    This is the only place other than set_state() allowed to call fire().
    """
    fire()
    print(f"[mist] pulsed - the relay should now be {STATE['believed']}, "
          f"which is what the dashboard shows")


# ---------------------------------------------------------------------------
# Config, mirroring sensorVPD/configReader.py
# ---------------------------------------------------------------------------

def _parse_text_config(text):
    """`key: value` per line. Split on the FIRST colon only, so a value
    containing one survives. Blank lines and # comments are skipped."""
    data = {}

    for raw in text.splitlines():
        line = raw.strip()

        if not line or line.startswith("#"):
            continue

        key, separator, value = line.partition(":")

        if not separator:
            continue

        key = key.strip()

        if key:
            data[key] = value.strip()

    return data


def _number(value, fallback):
    try:
        return type(fallback)(value)
    except (TypeError, ValueError):
        return fallback


def _load_config_data():
    """The file, parsed. JSON is tried first, so either format works."""
    raw = CONFIG_FILE.read_text()

    try:
        return json.loads(raw)
    except ValueError:
        return _parse_text_config(raw)


def read_gpio():
    """Just the pin name.

    Separate from read_config() and deliberately forgiving: a bench run of
    `relay_control.py 60` should still work on a Pi where config.txt has not
    been filled in yet, and the pin is the one setting that run genuinely
    needs. Everything else read_config() validates is about registration.
    """
    try:
        data = _load_config_data()
    except OSError:
        return DEFAULT_GPIO

    return data.get("GPIO") or data.get("gpio") or DEFAULT_GPIO


def read_config():
    """JSON or `key: value` text, same as the sensors."""
    try:
        data = _load_config_data()
    except OSError as e:
        raise SystemExit(f"[mist] cannot read {CONFIG_FILE}: {e}")

    actuator_type = data.get("Type") or data.get("actuatorType")
    location = data.get("Location") or data.get("locationName")

    # /api/registerActuator rejects a payload without these, and an actuator
    # that never registers just polls nothing forever. Failing here names the
    # file instead of leaving a 400 to be traced back from the dashboard.
    if not actuator_type or not location:
        raise SystemExit(
            f"[mist] {CONFIG_FILE}: 'Type' and 'Location' are required "
            f"(got Type={actuator_type!r}, Location={location!r})"
        )

    return {
        "deviceUUID": device_uuid(),
        "actuatorType": actuator_type,
        "locationName": location,
        "actuatorName": data.get("Name") or data.get("actuatorName"),
        "description": data.get("description") or data.get("Description"),
        "gpio": data.get("GPIO") or data.get("gpio") or DEFAULT_GPIO,
        "pollSeconds": _number(data.get("Poll"), DEFAULT_POLL_SECONDS),
        "maxRunSeconds": _number(data.get("MaxRun"), DEFAULT_MAX_RUN_SECONDS),
    }


def device_uuid():
    if not UUID_FILE.exists():
        UUID_FILE.write_text(str(uuid.uuid4()))

    return UUID_FILE.read_text().strip()


# ---------------------------------------------------------------------------
# Backend
# ---------------------------------------------------------------------------

def find_backend():
    """First address in networkList.txt that answers /api/time."""
    try:
        lines = NETWORK_LIST.read_text().splitlines()
    except OSError:
        print(f"[mist] cannot read {NETWORK_LIST}")
        return None

    for line in lines:
        host = line.strip()

        if not host or host.startswith("#"):
            continue

        url = f"http://{host}:{BACKEND_PORT}"

        try:
            if requests.get(f"{url}/api/time", timeout=3).status_code == 200:
                return url
        except requests.RequestException:
            pass

    return None


def register(base_url, config):
    r = requests.post(
        f"{base_url}/api/registerActuator",
        json={
            "deviceUUID": config["deviceUUID"],
            "actuatorType": config["actuatorType"],
            "locationName": config["locationName"],
            "actuatorName": config["actuatorName"],
            "description": config["description"],
        },
        timeout=5,
    )
    r.raise_for_status()
    return r.json().get("actuatorID")


def report():
    """Tell the backend what the hardware is actually doing. Doubles as the
    heartbeat - /api/actuatorState stamps lastHeartbeat on every call."""
    if LINK["base_url"] is None or LINK["actuatorID"] is None:
        return

    requests.post(
        f"{LINK['base_url']}/api/actuatorState",
        json={"actuatorID": LINK["actuatorID"], "state": STATE["believed"]},
        timeout=5,
    )


# ---------------------------------------------------------------------------
# Serve mode
# ---------------------------------------------------------------------------

def apply_command(cmd, max_run, off_at):
    """Act on a command we have not seen before. Returns the new off_at."""
    action = cmd.get("action")

    if action == "ON":
        set_state("ON")
        # Every ON gets an end time. There is no run-forever mode, and this one
        # rule is also the entire network-failure story: if the backend
        # vanishes mid-run, the timer still fires and the mister still stops.
        requested = cmd.get("durationSeconds") or max_run
        return time.monotonic() + min(_number(requested, max_run), max_run)

    if action == "OFF":
        set_state("OFF")
        return None

    # SET_SPEED belongs to the fan. Return off_at unchanged rather than None -
    # clearing a live timer here would strand the relay ON with nothing left to
    # switch it off.
    print(f"[mist] ignoring action {action!r}")
    return off_at


def _shutdown(signum, frame):
    """Never leave the mister running because the service stopped."""
    print(f"[mist] signal {signum} - stopping")

    try:
        set_state("OFF")
        report()
    except Exception as e:
        print("[mist] shutdown report failed:", e)

    sys.exit(0)


def serve():
    config = read_config()
    poll = config["pollSeconds"]
    max_run = config["maxRunSeconds"]

    print(f"[mist] {config['actuatorType']} @ {config['locationName']}, "
          f"pin {config['gpio']}, poll {poll}s, max run {max_run}s")

    # The board keeps its physical state across a restart, so a state file
    # saying ON means it is probably still misting. Converge before doing
    # anything else - this is what makes Restart=always safe.
    if STATE["believed"] == "ON":
        print("[mist] state file says ON after a restart - forcing OFF")
        set_state("OFF")

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    off_at = None
    last_report = 0.0

    while True:
        # Before the network, always: the auto-off has to fire whether or not
        # the backend is reachable.
        if off_at is not None and time.monotonic() >= off_at:
            print("[mist] run finished")
            set_state("OFF")
            off_at = None
            last_report = 0.0

        try:
            if LINK["base_url"] is None:
                LINK["base_url"] = find_backend()

                if LINK["base_url"] is None:
                    print("[mist] backend unavailable")
                    time.sleep(poll)
                    continue

                print("[mist] backend:", LINK["base_url"])
                LINK["actuatorID"] = None

            if LINK["actuatorID"] is None:
                LINK["actuatorID"] = register(LINK["base_url"], config)
                print("[mist] registered actuatorID:", LINK["actuatorID"])

            cmd = requests.get(
                f"{LINK['base_url']}/api/actuatorCommand",
                params={"actuatorID": LINK["actuatorID"]},
                timeout=5,
            ).json()

            if cmd.get("actionID") != STATE["lastActionID"]:
                STATE["lastActionID"] = cmd.get("actionID")
                off_at = apply_command(cmd, max_run, off_at)
                save_state()
                last_report = 0.0

            if time.monotonic() - last_report >= HEARTBEAT_SECONDS:
                report()
                last_report = time.monotonic()

        except requests.RequestException as e:
            # Network trouble only. Anything else is a real bug: let it crash
            # and let systemd restart us clean.
            print("[mist] backend error:", e)
            LINK["base_url"] = None

        time.sleep(poll)


if __name__ == "__main__":
    # Arguments first, before reading any file, so a usage error is not buried
    # under startup chatter about missing config or state.
    #
    # No default mode on purpose. The original single-mode script treated a
    # bare call as "mist for 60 seconds", which is now a trap: someone
    # expecting the service types `relay_control.py`, gets a silent 60-second
    # run, and sees no attempt to reach the backend. Nothing here touches the
    # hardware unless it was asked to.
    if len(sys.argv) < 2:
        raise SystemExit(USAGE)

    arg = sys.argv[1]
    seconds = None

    if arg not in ("serve", "flip"):
        try:
            seconds = float(arg)
        except ValueError:
            raise SystemExit(f"[mist] unknown mode {arg!r}\n\n{USAGE}")

        if seconds <= 0:
            raise SystemExit(f"[mist] run length must be positive, got {seconds}")

    # Every mode, not just serve: a bench run must pulse the same pin the
    # service does.
    GPIO_PIN = read_gpio()

    load_state()

    if arg == "serve":
        serve()
    elif arg == "flip":
        flip()
    else:
        run_for(seconds)
