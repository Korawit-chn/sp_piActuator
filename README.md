# Mist maker

Relay-driven mist maker, controlled from the dashboard. Built to match the
sensor Pi clients — same `config.txt` format, same `device_uuid.txt`, same
`networkList.txt`, same register-then-poll shape.

| File | |
|------|--|
| `mist_trigger.py` | Fires exactly one trigger and exits. Takes the pin as an argument; nothing else in it should change. |
| `relay_control.py` | Everything else: manual runs, dashboard control, registration, status reporting. |
| `config.txt` | Type, location, poll interval, run cap. |
| `networkList.txt` | Candidate backend addresses, tried in order. |
| `deploy/mist.service` | systemd unit. |

## Running it

```
python3 relay_control.py 60      # mist for 60 seconds, then stop
python3 relay_control.py serve   # obey the dashboard (this is what systemd runs)
python3 relay_control.py flip    # one raw pulse - see "When the relay is out of sync"
```

The pin comes from `GPIO:` in `config.txt` and is passed through to
`mist_trigger.py`, which also runs standalone if you want to test the board
without any of the rest:

```
python3 mist_trigger.py          # one pulse on D17, the default
python3 mist_trigger.py D27      # one pulse on D27
```

`read_gpio()` resolves the pin for every mode, not just `serve`, so a bench run
hits the same pin the service does. It falls back to `D17` if the key or the
whole config file is missing.

No new dependencies: `requests` is already in the sensor venv.

```
source ~/venv/bin/activate
python3 relay_control.py serve
```

## The board toggles — why that shapes everything

The relay board is edge-triggered and latching. It has no "set ON" and no
"set OFF", only a single input that flips it to whatever it was not, and there
is no way to read back what state it is in.

The dashboard is the opposite: `GET /api/actuatorCommand` re-serves the latest
command on every poll. Acting on that directly would pulse the relay every few
seconds and the mister would strobe.

So `relay_control.py` keeps a **believed state** in `mist_state.json` and fires
only on a transition. `set_state()` is the only function allowed to trigger the
board in normal operation. If you add code that calls `fire()` directly, the
belief drifts away from the hardware and stays wrong.

`mist_state.json` also holds `lastActionID`. A command is applied once and
never re-applied — without that, the stale `ON` sitting at the top of the log
would undo the Pi's own auto-off on the very next poll.

## Safety

Three rules, none of which need the network:

1. **Every run has an end time.** There is no run-forever mode. An `ON` gets
   either the duration the dashboard asked for or `MaxRun` from `config.txt`,
   whichever is smaller. This is also the whole network-failure story: if the
   backend disappears mid-run, the timer still fires.
2. **Start-up converges to OFF.** If `mist_state.json` says `ON`, the board is
   probably still misting from before the restart, so the service fires once
   and reports OFF. This is what makes `Restart=always` safe.
3. **Clean shutdown turns it off.** SIGTERM fires a last pulse if the relay is
   believed ON. The unit allows 20 s for it — a stop that gets SIGKILLed would
   leave the mister running.

Set `MaxRun` from how long the unit tolerates running dry, not from how long a
typical run is. It is the outer bound on every failure mode above.

**Do not run `serve` and a manual `relay_control.py 60` at the same time.**
Manual runs go through `set_state()` so the belief stays correct, but two
processes racing on one state file and one relay is still a mess.
`sudo systemctl stop mist` first.

## When the relay is out of sync

A missed pulse, a power glitch, or someone pressing the button on the board
itself will leave the belief wrong, and with no readback nothing can detect
that. The dashboard will confidently show the opposite of what the mister is
doing.

With only two states, a disagreement is always exactly **one toggle** out. So
one raw pulse fixes it, and it is the hardware that moves — the belief is what
the dashboard is already showing.

```
sudo systemctl stop mist
python3 relay_control.py flip     # one pulse, belief left alone
sudo systemctl start mist
```

`flip` deliberately does **not** change the stored belief. Pulsing *and*
inverting would move both and leave them just as far apart:

| | believed | actually |
|---|---|---|
| drifted | OFF | ON |
| after the pulse | OFF | OFF — correct |
| if it also inverted | ON | OFF — broken again |

If `mist_state.json` is missing entirely, the script assumes OFF and says so on
startup. That is the only assumption that cannot itself turn the mister on, but
it is an assumption — check the hardware.

## Device UUID

`device_uuid.txt` means the same thing here as on the sensor Pis: **one UUID
per physical Pi.** It is created on first run if absent.

If this Pi also runs sensors, link the two so the box has one identity in both
tables — **before the first start**, or a second UUID has already been minted
and registered:

```
ln -s "../PI sensor/device_uuid.txt" device_uuid.txt
```

Worth knowing: a sensor is identified by `(deviceUUID, typeID, locationID)`, so
one Pi can own several `Sensor` rows. `Actuator.deviceUUID` is `UNIQUE` on its
own, so **one Pi can own exactly one actuator row.** Fine for the mist maker
today. The day this Pi also drives the fan, the second registration will
silently land on the mist row — the fix then is a composite key on `Actuator`,
not a second UUID file.

## Running as a service

```
sudo cp deploy/mist.service /etc/systemd/system/
sudo nano /etc/systemd/system/mist.service     # check WorkingDirectory and the venv path

sudo systemctl daemon-reload
sudo systemctl enable --now mist.service
```

Check it:

```
systemctl status mist
journalctl -u mist -f
```

## What it talks to

| Call | When |
|------|------|
| `GET /api/time` | backend discovery, one per address in `networkList.txt` |
| `POST /api/registerActuator` | once, on first contact — find-or-create by `deviceUUID` |
| `GET /api/actuatorCommand` | every `Poll` seconds |
| `POST /api/actuatorState` | on every state change, and at least once a minute as a heartbeat |

Nothing here reads or writes a timestamp, so unlike the sensors this does not
need `sensor-timesync.service` and a wrong clock cannot misfire the relay.

`ActuatorLog` is written by the backend when a command is issued, not by the
Pi — there is nothing to upload and no offline cache. A command missed during
an outage is stale by the time the link is back, so there is nothing worth
replaying either.

## Automation

Climate rules exist in the database (`ClimateRules`, `ClimateRuleActuator`) but
nothing evaluates them yet. When that lands, this script needs no change: it
obeys the latest command regardless of `triggerSource`, so `AUTOMATION` rows
are picked up by exactly the same poll that handles `MANUAL`.
