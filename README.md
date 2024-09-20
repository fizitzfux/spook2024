# spook2024
The source code of my 2024 vacation project for a hayride.

```
It may look like a mess,  
it probably is a mess,  
the event it was for  
is over so I don't  
give a shit.
```

Do with it what you want, and feel free to ask questions in `Issues`, but I won't be updating this.

# IMPORTANT
The music used IS NOT properly licenced. It is a mix of:  
[Paxton](https://open.spotify.com/track/4hCtO2cNKBTkVhogXpJlpB)  
[Don't Be Scared Stella](https://open.spotify.com/track/4yKHRqu3A7ubBfKeG2OUgP)  
And a few free and open sound effects I found.

# Description
This is a program meant to run on a Raspberry Pi Pico (RP2040) microcontroller.
It drives the onboard LED and a LED lighting strip @3.3V using PWM on GPIO pin 13 with a maximum current of about 40mA.
If your strip uses too much current you can place a resistor at either end to reduce current flow.
Also using PWM it outputs analog audio on GPIO pin 16.
Between your audio device and the pin should at least be a 1K resisitor, and a 10-100uF capacitor is reccommended as well.

During runtime the board will play the audio and fade the lights in and out, applying effects randomly according to the weights in `values.rs`.

This system was purely used to distract the wanderers from the actual scare that was planned out, *and it worked well*.
