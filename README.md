# Notes
- TC3 is a timer counter
- multiplexing must 
- `candy_sync` makes sure frame is in-sync w/ message
- ODR: Output Data Register
- SODR: Set Output Data Register
- after SODR pull it down again with delay
- in second handler, call ODR and PDR
- `PA0` - `CANTX0`, `PA1` - `CANRX0`
- bit hammering is entirely dependent on `attachInterrupt` (runs `CANdy_sync`, which allows for `CANdy_hammer`/`CANdy_write` to be run) and `sendData` (sending a CAN frame) correctly working
	+ `attachInterrupt` does not correctly detect a `FALLING`/`RISING` edge and runs indiscriminately
