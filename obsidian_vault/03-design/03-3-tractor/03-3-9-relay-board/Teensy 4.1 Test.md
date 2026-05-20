Teensy 4.1 Test

![[Exported image 20260511103943-0.png]]  

Using VSCode:  
$ cd C:\Users\al532\OneDrive\Documents\tractor2025repo\teensy_blink  
$ platformio run --target upload  
$ pio device monitor --port COM3 --baud 9600 # this worked on W10
 
From C:\Users\al532\OneDrive\Documents\Robotics\layout JRK G2 21v3.pptx

![[Exported image 20260511103944-1.png]]  
![[Exported image 20260511103945-2.png]]

[https://www.amazon.com/dp/B00KTEN3TM](https://www.amazon.com/dp/B00KTEN3TM)
 
Chat in GPT regarding the relay: [Teensy E-stop Wiring Guide](https://chatgpt.com/c/685f11c5-a560-8013-aa63-c22dd9ecbd8b)
   

Test code:  
[https://github.com/jones2126/tractor2025/tree/main/tractor/testing/teeensy41_relay_test](https://github.com/jones2126/tractor2025/tree/main/tractor/testing/teeensy41_relay_test)
 
|   |
|---|
|$ cd ~/tractor2025  <br>$ git pull origin main  <br>$ cd /home/al/tractor2025/tractor/testing/teeensy41_relay_test  <br>$ pio run --target upload|
   
![[Exported image 20260511103947-3.png]]  

Looking at your close-up image of the K1 relay terminals, from left to right the three screw terminals are:  
**Left**: **NC** (Normally Closed) **Center**: **COM** (Common) **Right**: )**NO** (Normally Open)  
**Relay OFF (not energized, LED off):** Center (COM) to Left (NC): Should have continuity  
**Relay ON (energized, LED on):** Center (COM) to Right (NO): Should have continuity￼￼  
My relay operates this way:  
When the LED is NOT illuminated, there is continuity between the center screw and the left screw; The voltage on the signal line is 3.21 volts; Pin is set High.  
When the LED is illuminated, there is continuity between the center screw and the right screw; The voltage on the signal line is 0.0 volts; Pin is set Low