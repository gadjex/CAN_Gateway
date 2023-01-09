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
### Pinout:
![Terminating Resistors](/pictures/term_resistors.png)

The 120ohm terminating resistors come already connected with a small trace. If they are not needed you will need to cut the trace (RED LINE).

### Size:
PCB: 5cm x 3cm