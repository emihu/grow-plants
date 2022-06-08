# Grow Plants

A plant growing animation written in C and connected to a DE1-SoC computer system.
May be simulated on the [CPUlator Computer System Simulator](https://cpulator.01xz.net/?sys=arm-de1soc).

## Features
The animation is displayed on the VGA and can be interacted with using the push keys on the DE1-SoC board and mouse clicks with the PS/2. 

KEY0 clears all the plants on the screen.
KEY1 pauses the plant growth; clicking once will stop the growth and clicking again resumes it. 
KEY2 changes the speed of the plant growth; pushing this button loops through three preset speeds. 
KEY3 changes the colours of the plants.

Clicking on the screen with the mouse begins growing a new plant at the location of the mouse click.

<div align="center">
<img src="./assets/plants-1.gif" width=48% height=48%> <img src="./assets/plants-2.gif" width=48% height=48%>
<img src="./assets/plants-3.gif" width=48% height=48%> <img src="./assets/plants-4.gif" width=48% height=48%>
</div>
 
## Technical Features    
  * Double-buffering and v-synch to display on a VGA display  
  * Interrupt driven I/O to accept user input from PS/2 mouse and Push Buttons  
  * Interrupt driven I/O for tracking time with an Interval Timer <br><br>  
  * Linked lists of linked lists of structs to store plant information  
  * Various line drawing algorithms (quadratic Bezier, straight line, ellipse) paired with a time varying vector field for random velocities to animate plant growth  
  * Random colour selectors and plant branching algorithms for random plants  
