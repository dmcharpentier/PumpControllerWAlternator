# PumpController

Takes inputs for Hand/Off/Auto switchs to control up to 3 pumps using 1 or 2 pressure switches. Can be setup in a Lead/Lag configuration if using two pressure switches. If using one pressure switch with multiple pumps in Auto it will alternate. If you put a jumper between Input 7 and Input 8 the controller will run two pumps simultaneously. If it detects three pumps are in auto it will round robin in either alternating or Lead/Lag modes.

## Why?
Developing a pump control box with alternator and lead/lag capability. This particular code is for an arduino nano v3 with Eletechsup DN23E08 Expansion Board.

The Relays are paired to pumps as
Primary & Secondary Relays
Pump 1: Relays 1 & 8
Pump 2: Relays 2 & 7
Pump 3: Relays 3 & 6
Aux/Chem: Relay 5 will always run if a pump is running

## Variables and pins
All pins are defined and the code is heavily commented. Please take a look and submit an issue if you find one! Pull requests are always welcome with improvements :)