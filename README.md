## CAN Gateway with STM32F105RBT
Repurposing a cheap board that goes by the name "MB CAN Filter" on AliExpress. This board is marketed for shady things to do with vehicle odometers but it is a very useful board with a STM32F105RBT6 microcontroller, 2 CAN interfaces, 2 120ohm terminating resistors and a 7-30 volt input (if you want to use it with a 5 volt input, you can remove the 5 volt regulator and bridge the positive input and output pins). 3 pads also expose the USART3 synchronous interface with a bitrate up to 2.25 Mbit/s. The two CAN interfaces are compliant with the 2.0A and B (active) specifications with a bitrate up to 1 Mbit/s.

The master branch contains example code I am using to relay certain message IDs from 2 seperate CAN buses operating at different bitrates.

### Hardware
![Front](/pictures/front.png)

![Back](/pictures/back.png)
 
### Pinout:
![Pinout](/pictures/pinout.png)
```
CAN1 is assigned to PA11 & PA12
CAN2 is assigned to PB05 & PB06
There are 4 GPIO pads that can be useful for other tasks or use a solder bridge to the exposed ground bar for option selection:
 - PA15: labeled "BMW"
 - PC10: labeled "W166" (USART3 TX)
 - PC11: labeled "W222" (USART3 RX)
 - PC12: not labeled (USART3 CLK)
There is also a convenient programming and debugging header.
```
### Terminating Resistors:
![Terminating Resistors](/pictures/term_resistors.png)

The 120ohm terminating resistors come already connected with a small trace. If they are not needed you will need to cut the trace (RED LINE).

### Size:
PCB: 5cm x 3cm


## 2021 - 2023 Honda Ridgeline OpenPilot Modifications
The CAN Gateway needs to be connected to the body CAN of the Ridgeline and CAN0 on OpenPilot. This can be accomplished many ways including hooking into the body CAN wires under the dash and using a 3-way splitter on the comma power OBD-II port adapter where you can find CAN0. If your Ridgeline is equipped with auto high-beams, the body CAN is already under the windshield camera cover and is part of the OpenPilot camera harness.

### CAN Bus Connections
![Modified Nidec Harness](/pictures/modified_nidec_harness.jpg)
I made both of the connections to my CAN Gateway under the windshield camera cover. On the [windshield camera connector](documents/Ridgeline_Multipurpose_Camera_Unit_Connector(20-23).pdf) pins 6 and 17 are CAN-High and CAN-Low respectively. These feed into the 26-pin connector on the relay box on pins 7 (CAN-High) and 9 (CAN-Low) as PT4 and PT3 respectively. CAN0-High is pin 4 and CAN0-Low is pin 6. I used a 6 pin connector to make the CAN Gateway removable.

![Harness Connections](/documents/harness_connections.png)


The method I used to tap into these wires included stripping and removing a small piece of the insulation, wrapping a stripped end of wire around it, soldering the wires together, and covering with adhesive heatshrink. This method does not cut the wire to maintain its integerity.

![Stripped Wire](/pictures/wire_stripped.jpg)
![Soldered Wire](/pictures/wire_soldered.jpg)
![Heatshink Wire](/pictures/wire_heatshrink.jpg)


### Preparing the CAN Gateway
- Use a programmer such as a ST-Link and STM32CubeIDE to program the CAN Gateway.
- Cut the small traces to disconnect the terminating resistors as [shown above](#terminating-resistors).
- Cover the PCB. I used large heatshrink.

![Prepared CAN Gateway](/pictures/gateway_prepared.jpg)

### Preparing OpenPilot
[You will need to make changes to the OpenPilot code to make this work](https://github.com/gadjex/openpilot/commit/cee16d28efe93724ecf8f3a1cf2935eba8f1b611). At this time a pull request has not been made or accepted into the main repo. Make sure to disable updates or even better fork the OpenPilot repo and maintain your own fork. [I also have a fork dedicated to the 2021-2023 Honda Ridgeline](https://github.com/gadjex/openpilot/tree/ridgeline) you can install with this URL:
```
installer.comma.ai/gadjex/ridgeline
```
