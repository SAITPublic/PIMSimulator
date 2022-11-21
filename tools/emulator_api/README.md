# emulator_api

In emulator mode of PIM Kernel(in PIM SDK), it records memory trace data instead of executing PIM Kernel on real machine.
The emulation can then proceed by inserting a memory trace into the PIM simulator as an input.

`emulator_api` consists of source codes that support the emulation path of the PIM kernel.
These codes for the emulation API are built by default at compile time.

However, if you do not want to build with these codes, compile as follows:

```bash
scons NO_EMUL=1
```

