# CANdy
The following directories (and their sketches) are currently in use. Below is a short description of the respective sketches.

## [`sender`](./sender/sender.ino)
- sends the data specified (e.g., `0x0000000000000000`) utilizing the `due_can` library every second
- mainly used to test `hammer2`
- toggles LED after successful sending of frame

## [`receiver`](./receiver/receiver.ino)
- receives frame on CAN bus on `CAN0` utilizing the `due_can` library and prints out the frame
- mainly used to test `hammer-integrated`
- toggle LED after reception of frame

## [`hammer-integrated`](./hammer-integrated/hammer-integrated.ino)
- sends out frames on `CAN0` hammered (starting on data bits) with HMAC of data sent
- solely utilizes pin manipulation and standard `can_` functions

### Workflow
- sets ID, data length, and data for frame
- calculates the HMAC for data and bits until data bits (counting stuff bits)
- sends out data utilizing `can_global_send_transfer_cmd` with mask for available mailbox
- listens for `CAN_TSTP` interrupt on `CAN0` and starts if frame sent
- `CAN0_Handler` starts `TC3` at ~0% mark for first data bit
- `TC3_Handler` starts `TC4` to run at ~0% mark for subsequent data bits and `TC6` to start at ~20% for that data bit
  + `TC6_Handler` writes one hammered bit and starts `TC7` to write subsequent hammered bits at 10x of bus speed
    * `TC7_Handler` hammers subsequent bits for that specific data bit and stops by writing the reset value (original value)
- `TC4_Handler` again starts `TC6` (and subsequently `TC7`) until all the bits are written

## [`hammer2`](./hammer2/hammer2.ino)
- acts as "modem" to hammer data sent by its associated sender ECU
- bridges connection from sender ECU to the other ECUs by operating two CAN buses
  + `CAN0` used to receive frames from sender (and replicate messages from other ECUs)
  + `CAN1` used to send out hammered frames (and receive messages from other ECUs)

### Workflow
#### `CAN0`
- listens for `CAN_TSTP` interrupt on `CAN0` and starts if frame sent
- `CAN0_Handler` starts `TC0` at ~70% mark for second data bit (starts instantly) and writes SOF to `CAN1`
- `TC0_Handler` writes `frame_value` (old sample) and resamples; it also starts `TC3` when the data bits have started at ~20%
    + `TC0` writes the SOF again for the first iteration
- `TC3_Handler` starts `TC4` to run at ~0% mark for subsequent data bits and `TC6` to start at ~20% for that data bit
  + `TC6_Handler` writes one hammered bit and starts `TC7` to write subsequent hammered bits at 10x of bus speed
    * `TC7_Handler` hammers subsequent bits for that specific data bit and stops by writing the reset value (original value)
- `TC4_Handler` again starts `TC6` (and subsequently `TC7`) until all the bits are written

#### `CAN1`
- `CAN1_Handler` starts `TC1` at ~70% mark for second data bit and writes SOF to `CAN1`
- `TC1_Handler` is used to copy over messages sent from `CAN1` to `CAN0` (has a one bit delay)
