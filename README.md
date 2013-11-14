# Offroad CPU

Inclinometer, altimeter and compass for your offroading pleasure. Circuit board
and parts list can be found here: https://github.com/camerontech/offroad-cpu-hardware

If you're building your own you can pick up the [Cameron Tech circuit board](http://store.camerontech.io/products/offroad-cpu-circuit-board) and
[three-way nav switch](http://store.camerontech.io/products/three-way-rocker-switch).
You'll probably also want an [ATMEGA chip](http://store.camerontech.io/products/atmega) with the Arduino bootloader ready to go. If you order one and ask nicely I'll even pre-program
it with the latest software from the master branch here.

Finally, if picking up and assembling all of these parts yourself is a bit
daunting, you can purchase a kit or pre-assembled unit from Cameron Tech (coming
soon).

## Development & Branches

You want to use the `dev` branch when working on an actual Arduino board. It
uses slightly different pin mappings for the buttons so that it doesn't clash
with the LED on pin 13.

The `oled` branch contains a different display library for working with OLED
screens.

Generally you will want to make changes in the `dev` branch and then merge them
into `master` if/when they are ready to be put on a standard [Cameron Tech
Offroad CPU circuit board](http://store.camerontech.io/products/offroad-cpu-circuit-board).
If you have an OLED screen you then want to merge your changes into the `oled`
branch.
