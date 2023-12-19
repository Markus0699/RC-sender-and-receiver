# Selfmade sender and receiver for an RC car
#### Video Demo: https://youtu.be/b8qKtN-1DAY
#### Description:
The project consists of two programs: code for the sender of the RC car en code for the receiver of the RC car. The sender code runs on a Teensy LC in the form of C++, Arduino based. The receiver runs on a Arduino Pro Mini in the form of C++ code, Arduino based. The sender (Teensy LC) reads all the inputs (buttons and joysticks) and sends the current status of all componentents constantly via a NRF24L01 radio module. The sender also displays a menu with all kinds of car data via an oled display. Also a few leds are mounted in the controller to give the user feedback.
On the receiver (Arduino Pro Mini) is also a NRF24L01 radio module connected, which the microcontroller constantly reads. It converts all input to commands for the steering servo and motorcontroller. It also turns led strip on and of, mounted on the front and back, a buzzer is also mounted. The user can read the status of the receiver via a bunch of leds.

### Coming up with the idea
So two summers ago I started designing the hardware for a selfmade RC car. For that I used a 3D-printer which I recently bought. After completing the hardware (chassis and body), I also wanted to design my own remote control, so I could truly say everything on the RC I designed myself. I used this opportunity to make this happen. I used the remote control I bought to check if it was possible to control the motorcontroller via a microcontroller. For that I used an oscilloscoop. After examening the signal I saw that the signal is just an regularly used PMW-signal, so I made a quick and dirty test to see if I could manipulate the motorcontroller with a microcontroller, and yes I succeeded, which kicked of this project immediately.

### Software designs
The project presist out of two programs designed in PlatformIO based on C++ with Arduino flavour. Both programs operate via a statemachine which calls all kinds of functions to complete its tasks. The programms are quite packed and not very readable in my opinion, in the future I want to improve that, more on that later.

### Hardware designs
For this project I also designed and realised two PCB's, one for the sender and one for the receiver. Those PCB's act as a sort of motherboard where everything plugs into (microcontroller, switches, joysticks, oled display, motorcontroller, servo, leds, power). The PCB's have internal power regulators for powering everything via batteries.
I also designed a housing for my own remote controller, after that I 3D-printed the design.

### Testing
For now everything has been quickly tested. As of now everything looks to be working correctly, I still need to find some spare time to go outisde to properly test everything and fix some upcoming bugs.

### Improvements
As I said earlier, the programs are quited cluttered and unreadable, but everythin functions correctly. I'm still looking for ways to improve realibility and readability. I think reprogramming everythink in OOB might help the situation.

### Future
I"m still in the early stages of developement, you can look at this like and first proof of concept. I need to thoroughly test the functionality outside of the RC car with the new controller. With the finding I make I'll probably have to do some adjustments and fine tuning. After that I want to reprogram everything to make the software more readable, I just need to find A LOT of time to do all these things...
