# PTA1
Robin Davies repository for Programming "Things" Assignment 1 (PTA1)

Feature list.

Task 1 - move the zumo with a gui. using buttons or a text field 

Task 2 - have the zumo continue to move forward until it hits a wall, then stop without crossing the wall 

Task 3 - Return the hit wall message to the gui 
         Deactivate auto behaviour from task 2
         Human controller then turns the zumo
         Signal the zumo to continue auto behaviour with another keypress (c for continue)

Task 4 - sending button press or text field data (eg. “Ro” for room and 'R' or ‘L’ for right/left). 
         They will then turn the robot towards the room.
         message should appear in the GUI, 
         giving that room a number and identifying whether the room is on the left or right of the corridor. 
         Zumo should then move into the room and do a 360 spin noting any object in the room and marking room number with objects  
         should stop and wait for the human controller to navigate the robot out of the room and turn back into the corridor
         The same keypress as that used to signal that a corner-turn is complete should signal that the robot is back in the corridor and being driven as in Task 2. 

Task 5 - Turn either way on the T junctions. 
         Doing everything that needs to be done on the way (room turns etc)
         Press e when reached the end, to then travel back the the junction
         ignoring any instructions to turn into the room or down the main corridor, 
         (These instructions must be sent though, so that the robot knows it has passed those points) I dont get this

Task 6 - At the end of the second side of the t junction
         Second "e" press 
         Zumo goes home checking rooms that had objects
         if the object is gone, good 
         else put on a light pin 13.
         when reached the end light turns off and resets.

GUI needs to be able to take messages have a wasd movement scheme
a button to check room
a button to continue 
a button to turn back on the t junction 
    this same button should travel home after 2nd press

Task 6 will need somethinking