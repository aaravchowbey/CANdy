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
	+ ~~`attachInterrupt` does not correctly detect a `FALLING`/`RISING` edge and runs indiscriminately~~
	+ `attachInterrupt` does not work on `CANTX0` and does work on `CANRX0` (`CANRX0` will end up mirroring `CANTX0` so it works!)
- **USE `PIO_PDR` for `CANTX0`** not `PIO_PER`

## 2024-04-21
- `CANdy_Sync` not running consistently indicates the problem has to be with reading `CANRX`
  + tested by turning off sender and using `attachInterrupt` with `LOW` (it still triggers)
