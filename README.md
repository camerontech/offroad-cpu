# Offroad CPU

Inclinometer, altimeter and compass for your offroading pleasure. Circuit board and parts list can be found here: https://github.com/camerontech/offroad-cpu-hardware

If picking up and assembling all of these parts yourself is a bit daunting, you can purchase a kit or pre-assembled unit here: http://store.camerontech.io/products/offroad-cpu

Documentation coming soon...

## Working with Branches

I use a bread-boarded version of the components while developing. `master` contains
the proper nav switch pins for my breadboarded version. `led` contains the real
production pin numbering. `oled` contains the same production pin number plus a
different library for talking to the screen.

So a typical development task will involve making changes to `master`. Once those
look good and are ready for production they are merged into `led` and `oled` and
will be burned onto the next set of production ATMEGAs that go out the door.
