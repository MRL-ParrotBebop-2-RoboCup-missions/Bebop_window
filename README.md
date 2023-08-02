
# Bebop_window



In this task, We are trying to pass the bebop through the window which has been colored.Bebop must perform the following sub-tasks respectively.


## Features

- Window Detection
- pid Controller (decreasing errors in x,y,z dimensions)
- pid controller (adjusting Drone speed)
- passing through the window

## Window detection 

Drone can detect special objects aiming its camera or sensors.
In this task We utilize opencv Library and apply HSV transform to detect the blue color of the window.Then, having the position of the robot and the difference between the center of the image and the middle of the window, we can adjust the position of the robot by giving the difference numbers to the controller.


Full tutorial :

https://www.linkedin.com/feed/update/urn:li:activity:7030598976512892928?utm_source=share&utm_medium=member_desktop
##  the Proportional-Integral-Derivative (PID) controller 

When We have position errors in x,y,z dimensions, We simply can use pid controller in our project to decrease those errors . 

Tutorial :
https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwiAtZnK372AAxWDTaQEHUZ6DGMQFnoECBcQAw&url=https%3A%2F%2Fwww.omega.com%2Fen-us%2Fresources%2Fpid-controllers&usg=AOvVaw0uxGPDA38LGDhXTbVxHkeA&opi=89978449
