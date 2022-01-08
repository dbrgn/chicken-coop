# Chicken Door

An automated door for our chicken coop.

## How it works

The main goal of this project is to automatically open the door in the morning
(using a gear motor), and to close it at night.

A combination of RTC and ambient light sensor is used:

- In the morning, the door will open if the `OPENING_LUX_THRESHOLD` is passed,
  but not before `EARLIEST_OPENING_TIME`.
- If `LATEST_OPENING_TIME` is reached, the door will open even if the
  `OPENING_LUX_THRESHOLD` has not yet been reached.
- Once the door is open, it will not close again before `EARLIEST_CLOSING_TIME`
  is reached.
- After `EARLIEST_CLOSING_TIME`, the door will close once the ambient light
  falls below the specified `CLOSING_LUX_THRESHOLD`.
- If the `CLOSING_LUX_THRESHOLD` is not reached at `LATEST_CLOSING_TIME`, the
  door will close anyways.

To detect an open or closed door, reed switches are used.

### State machine

    ┌───────┐                         ┌─────┐
    │Initial├────────────────────┐    │Error│
    └───────┘                    │    └─────┘
       │                         │
       ▼                         │
    ┌──────┐     ┌──────────┐    │
    │Closed├────►│PreOpening│    │
    └──────┘     └───────┬──┘    │
       ▲                 │       │
       │                 ▼       │
    ┌──┴───────┐       ┌────┐    │
    │PreClosing│◄──────┤Open│◄───┘
    └──────────┘       └────┘

- `Initial`: The state when turned on. The controller will transition to the
  "Open" or "Closed" state depending on the sensor readings.
- `Closed`: Waiting for `EARLIEST_OPENING_TIME`
- `PreOpening`: Waiting for either `OPENING_LUX_THRESHOLD` or `LATEST_OPENING_TIME`
- `Open`: Waiting for `EARLIEST_CLOSING_TIME`
- `PreClosing`: Waiting for `CLOSING_LUX_THRESHOLD` or `LATEST_CLOSING_TIME`
- `Error`: Reachable from any state when something goes wrong (e.g. reading a sensor)

## Testing

To run unit tests:

    cargo test --target x86_64-unknown-linux-gnu --tests

## Flashing

    cd firmware
    cargo flash --connect-under-reset --chip STM32F411CEUx --release

## Serial Terminal

To open a serial connection:

    python -m serial.tools.miniterm /dev/ttyACM0 9600

Then type `?` to see the help.
