# IR LED Panel

## Description

An IR LED panel with OSRAM SFH4258 LEDs.

## LED Calculation

- Source voltage: 12 V
- Diode forward voltage: 1.5 V (max 1.7 V)
- Diode max forward current: 80 mA (max 100 mA)
- LEDs: 10

Wiring:

      |-O->|O->|O->|O->|O->|--R---|   	R = 68 ohms
   o--|                           |--GND
      |-O->|O->|O->|O->|O->|--R---|   	R = 68 ohms

Each 68 ohm resistor dissipates 435.2 mW. This means 1 W resistors are
required.

- Together, all resistors dissipate 1395.2 mW.
- Together, the diodes dissipate 1200 mW.
- Total current drawn from source: 160 mA @ 12 V.

Calculated with: http://www.hebeiltd.com.cn/calculator/v5/led.php

## License

See [LICENSE.txt](LICENSE.txt).
