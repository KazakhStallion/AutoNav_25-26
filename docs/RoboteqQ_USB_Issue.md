# RoboteQ USB Issue — Clearing a Latched MOSFET Fault

When the **USB/status LED on the RoboteQ is red** and the robot will not move, the
controller likely has a latched fault flag. Its output stage is disabled until the fault is
cleared. This is how to confirm it and reset it.

## STEPS

Ensure you are inside the JETSON (host, not the container) when you are invoking these commands.
Stop the control node first so it isn't fighting you for the serial port.

- The command below defines a variable to hold the RoboteQ serial ID, and then shows any process on that port.
- KEEP IN MIND: The serial ID will change between controllers.

```bash
DEV="/dev/serial/by-id/usb-RoboteQ_RoboteQ_FBLG2360T_IwBCAAlQMTlZMDIg_2074307C3931-if00"
fuser -v "$DEV"
```

BTW if fuser returns a PID owned by cat, kill it. There could be multiple.
What it looks like:

```text
                       USER        PID ACCESS COMMAND
  /dev/ttyACM1:        vtcro      2507 F.... bash
                       vtcro      2646 F.... cat             
```

```bash
kill {PID1} {PID2}
```

Set 115200 baud, 8 data bits, no parity, 1 stop bit, flow control off, raw, no echo:

- The command below sets up a serial connection to that serial device which is the DEV variable you defined inline.

```bash
stty -F "$DEV" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw -echo
```

Start ONE reader in the background so the controller's replies are printed.
Without it, `printf ... > "$DEV"` sends the query but the reply is never shown.
(Kill it again when you're done — see the fuser step above.)

```bash
cat "$DEV" &
```

Next we want to just ensure that the MOSFET fault is correct.

- The commands below send a little message to the DEV serial variable and what prints out is what the controller responds with.
- NOTE: You don't need to copy the little comments.

```bash
printf '?FF\r' > "$DEV"   # Fault flags   -> FF=64  means MOSFET failure (latched)
printf '?FS\r' > "$DEV"   # Status flags
printf '?FM 1\r' > "$DEV" # Motor 1 runtime flags
printf '?FM 2\r' > "$DEV" # Motor 2 runtime flags
printf '?V 2\r' > "$DEV"  # Main battery voltage in 0.1 V (e.g. 259 = 25.9 V)
```

`FF` is a bit field (values add up, e.g. `FF=20` = 16 + 4). Check against the RoboteQ manual:

| Value | Fault |
|---|---|
| 1 | Overheat |
| 2 | Overvoltage |
| 4 | Undervoltage |
| 8 | Short circuit |
| 16 | Emergency stop — also set by the software `!EX` that the control node sends on B / `/estop STOP` |
| 32 | Motor/sensor setup fault |
| 64 | MOSFET failure |
| 128 | Default configuration loaded at startup |

Write down the reading BEFORE resetting — it is the only evidence of what tripped.

If you see `FF=64` then you need to reset the MOSFETS. Sending this RESET along with the controller's PASSWORD will reset it.
Note the doubled `%%`: `printf` treats a single `%` as a format specifier, so `'%RESET'` does not send the right bytes.

```bash
printf '%%RESET 321654987\r' > "$DEV"
sleep 3
```

## VERIFICATION

Re-read the fault flags — a successful reset returns everything to zero.
The reset drops the USB connection, so restart the reader:

```bash
stty -F "$DEV" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw -echo
cat "$DEV" &
printf '?FF\r' > "$DEV"   # expect FF=0
printf '?FM 1\r' > "$DEV" # expect FM=0
printf '?FM 2\r' > "$DEV" # expect FM=0
```

Kill the `cat` reader (see the fuser step) before starting the control node again.
