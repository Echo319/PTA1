# PTA1
Robin Davies repository for Programming "Things" Assignment 1 (PTA1)
Video is too large to add to the repository so please find it here: https://drive.google.com/open?id=1QGE4PYjZXwB-QABd6EqSd5_BCK7A7S1y
I will also include it in the zipped version of the submission.

I used the ZumoShield library, lifting alot of the compass code from the given example and using . 
Newping and Wire.h library. 
This java serial library - https://fazecast.github.io/jSerialComm/
A stack library from - https://playground.arduino.cc/Code/StackArray

### Feature list.

Task 1 - Move the zumo with WASD from keyboard and GUI 

Task 2 - Zumo will move within the walls stopping when hitting a wall

Task 3 - When Zumo hits a wall command is returned to the User and the User can adjust before pressing a continue button in the GUI

Task 4 - Zumo is now able to go into "rooms" on the left and the right, scanning for objects, and alerting if found.

Task 5 - Turning down the T-Junction, pressing E at the end of one side to return to the centre and prepare for the second half

Task 6 - Pressing E for the second time at the end of the other side of the junction. Takes the zumo home checking rooms that had objects in previously. (Extremely unreliable)

### Design
#### GUI
A GUI was created that allows for the keyboard to use wasd and 0 to move manually and cancel the auto move feature. start, stop, continue and end buttons were also added to account for the other behaviours. The GUI was made in NetBeans using Java, I just prefer it over the processing. I was able to import a library that connected me to the xBee and I am freely able to write and read from that serial. I pipe the input into a textPane in the middle of the GUI, so all messages come through. 
#### Script
The script went through several variations. Task 6 demanded a way to follow our footsteps, so the initial idea was to save every command with the time it was done to read back everything that was done. This however proved to be extremely unpredictable so was scrapped. The next plan was to use the onboard compass to provide location data. I discovered that it had x, y and z values and guessed they could be used to map out "nodes" that we can walk to. This proved to not be the case the values did not change in proportion to distance travels which lead to this not working. However, it is a compass. So, with the heading and the duration of the move it should be possible to retrace our steps. And it works. If the compass wants to play fair. In the video right at the end you can see the Zumo turn too far and travel into the neighbouring room. This is the most consistently wrong turn. I theorise that there is some magnetic interference there, potentially from the speakers or TV directly behind it (or maybe a steel bar in the floor?). If the compass was accurate or the x and y values were proportional maths would have saved the day, but the machine is not accurate enough to guarantee.

## Strategies and problems 
Tasks 1-3 were trivial, 4 came down to number tuning until the last few hours where NewPing has broken. Both my own code and the example code do not work, they consistently return 0 distance. Trying someone else’s kit proved it to be my library or my laptop as my kit would work with their example code and there’s will not with mine. I have no idea what has happened or how to remedy. The difficult bit about this being, I discovered the problem at 3am on Monday 28th January. 
Overall the variation from run to run, lead me to work through the compass library which provided the idea of nodes. The idea is good but the maths behind it is very accurate. So, when the compass varies as it travels. Some places on the course are perfect and others are way off. This has made Task 5 and 6 extremely difficult. 
The plan was if it were possible to record some location data relative to the starting point, we can math our way out of the problem. With direction, consistent speed and time should have been able to derive everything we would need. So, we would save "Locations" or nodes. These were originally going to use the x, y coordinates from the compass but as I played, I discover they are aren't true locations so that plan went out the window. The angle found by the compass however seemed rather accurate. So, we could get direction from the last location and the time spent traveling away from it. So, when it came to a return journey, we would simply turn to that direction then turn 180 degrees and travel for the same amount of time. These locations could then be saved onto a stack as the first node would be our last to return to (FILO).
Task 5 was pop off 1 (or 2 if that 1 was a room we would want to revisit) node and go to it. Overall simple, it performs well enough and rather consistently. The compass can provide strange results sometimes meaning it’s not 100% consistent. The position of the zumo, if the motors are running (maybe even the time of day) can change the compass values. On my course I can reliably return home from the middle of the T-junction however the compass is consistently wrong for the final turn. Which is odd.  
