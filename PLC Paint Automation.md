# Automated Paint Booth
## Description

This was a school project to simulate an automated paint booth in which a component is moved from an incoming feed platform, to the painting platform, and then to a drying platform where the part is then manually removed. to move between platforms an actuator drops down and uses suction to hold the part until it has been moved to the next platform. Suction is disabled when a load cell reaches a certain threshold which means that the part has made contact with a new platform. Clicking the image below takes you to the demo video but it seems like its too big to watch on github so you'll have to download it.
[![demo video](https://github.com/lukajuci/Engineering-Portfolio/blob/main/images/paint%20automation/plc%20paint.png)](https://github.com/lukajuci/Engineering-Portfolio/blob/main/images/paint%20automation/plc%20paint%20demo.mp4)

## Logic

This is the logic diagram I made before creating the code for the paint booth. 

![plc logic](https://github.com/lukajuci/Engineering-Portfolio/blob/main/images/paint%20automation/plc%20logic.png)

## Code

The PLC code for this project can be found [here](https://github.com/lukajuci/Engineering-Portfolio/tree/main/code/paint%20automation). In that folder is also the file for the HMI that I made for the simulation.
