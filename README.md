Project Caribou Writeup
=======================
*Mafalda Borges, Ian Hill, and Liani Lye*

ROS package that enables a Neato to recognize a stop sign and avoid other Neatos.

Video Demo: <link>

## What was the goal of your project?
In our CompRobo Computer Vision project, we wanted our robot to behave like a very simple self-driving car. We intended that the robot would:
follow a line on the floor (representing the “roadway”),
recognize signs along the roadway, and
“look both ways”  before crossing an intersection so as to not hit other Neatos.

## Describe how your system works.  Make sure to include the basic components and algorithms that comprise your project.

### Line following
The first part of our system allows the neato to follow the “road” which is comprised of white tape on the floor. 
The image was converted to grayscale.
A filter was added to select for white objects.
Only the bottom tenth of the image the camera can see is used.
The white pixels are averaged to find the the center point of the track.
The robot drove towards the found point.

### Sign recognition<a name=”sign-recognition”></a>
To recognize the signs, we performed multiple operations:
Colorspace was converted to hue, saturation, and value.
A filter that only selects red objects was applied.
A Gaussian blur was applied to the subsequent image, allowing for more continuous contours to be more readily found.
Canny edge detection was used to detect of the stop sign including the word stop.
The edges were merged together using the morphology transform morphologyEx to create closed polygons.
Contours were found using approxPolyDP.
The found contours were looped over and if the approximated contour has 8 points,we returned that an octagon was found.
All octagons that had an area below 10,000 pixels were ignored.

### Collision avoidance <a name=”collision-avoidance”></a>
Frames from the camera were taken every ⅓ of a second.
A cache of the last ten images were created.
Active points were compared.
If 60% of the active points matched descriptors it was determined nothing was moving in front of the bot and the bot continued forward. If not the bot stopped until the 60% tolerance was reached.

## Describe a design decision you had to make when working on your project and what you ultimately did (and why)?
*These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.*

Two critical design decisions revolve around sign recognition.  To effectively detect signs, we set out to combine three methods: color filtering, shape recognition, and text detection.

### Nixing Text Detection
Text detection using the [Tesseract library](https://pypi.python.org/pypi/pytesseract) was the first method implemented.  However, it proved to be extremely sensitive.  Tesseract worked best with minimal noise and almost perfect bounding-box placement.

Such perfect bounding-box placement proved more trouble than it was worth, especially as text recognition would prove almost wholly redundant and outright unnecessary if we were to add more street signs, such as one-way signs and turn signs, respectively.

![bounding with corners](/readme_images/bounding.jpg)

In the first diagram, the bounding box includes some of the sign’s perimeter, breaking the text-recognition.

In the second diagram, the bounding box clips the edge of the line of text, also breaking the text-recognition.

The third diagram illustrates the most ideal drawing of the bounding box, in which the text is nicely centered and no clipping occurs.  This is tricky, given the complexity of appropriately sizing the bounding box and the relatively small street sign.

Although we hadn’t successfully implemented color and shape recognition and therefore no working alternative to fall back on, we decided to move on without text recognition due to the challenges we would need to overcome to make it work as we wanted.

### SIFT vs Contouring

We dropped text-recognition in favor of shape-detection.  There were two options: contour detection or SIFT.  Contouring proved troublesome because the findContours method returned multiple broken contours - the perimeter of the sign consisted of 10 separate contours.  SIFT (Scale Invariant Feature Transform) proved troublesome because of all the false positives inherent in our noisy background.  We bounced back and forth between the two options, sinking time into implementing both poorly.  We finally landed on a procedure that resulted in a more continuous contour, and proceeded with contouring for shape recognition.  See [Sign recognition](#sign-recognition).  We instead found SIFT more appropriate for [collision avoidance](#collision-avoidance).


## How did you structure your code?

### Controller
The code centered around a finite state controller object which tracked the state of the Neato, received images from the Neato’s camera, and sent commands across the ROS network to control the Neato. The controller defined the robot’s behavior in each of its three states: driving, stopped, and looking-both-ways. It also initialized ROS and created the graphical interface to calibrate the image mask thresholds required to detect the roadway and signage.

The **react_to_image** method defined much of the behavior within the controller and used the helper functions to do most of the heavy-lifting required to calculate what the robot should do next. The three states can be clearly seen in this method where there is an **if** statement for each state.

In the **DRIVE** state, the robot uses the **find_line** helper function to detect the line and get a simple tuple describing where the line has been detected. The controller’s own **self.drive** method interprets that direction and sets the command to the robot. If a sign is detected in the **DRIVE** state, the robot switches to the **STOP** state and sets a brief timer.

When the timer returns, the robot switches to the **LOOK_BOTH_WAYS** state where the third **if** statement of **react_to_image** uses SIFT to make sure that nothing is in the robot’s way before crossing the intersection.

### Saving Settings

After wasting a good deal of time re-calibrating the masking bounds required detect certain features of the images being processed, we eventually began saving the bounds to a **settings.txt** file which is now read when the controller is initialized. When the program is stopped, the bounds set in the controller are saved back to the file.

### Helper Functions

The helper functions kept in a separate python module, **helper_functions.py**, performed the major calculations required to inform the controller’s decisions. It contained two major functions, **find_line** and **find_stop_sign**. **find_line** averages the positions of white pixels in a binary image to determine where the line in front of the robot is relative to 0 degrees in front of the robot. **find_stop_sign** uses contour extraction and polygon approximation to find stop signs in the robot’s path. 

### Style
At the beginning of this project, we made the decision to properly utilize branches and pull requests in Git as well as code according to our own custom style guide. Surprisingly, given the “chill” nature of CompRobo, we stuck to our guide with unusual discipline. Ultimately, we believe this made our code more readable and our repository more presentable.

## What if any challenges did you face along the way?

### Sign Recognition: SIFT vs contouring vs text recognition
Early on it was evident that HSV was needed for sign recognition but we found it difficult to determine was secondary function we should use. Initially we latched on the the idea of using both contours and text recognition. Text recognition was least pursued owing to the difficulty to implement and its lack of using when scaling to more signs. Most signs, especially warning signs, have no text so it would not be terribly useful if we wished to recognize more signs. Contouring initially was difficult. Without proper thresholding and filtering of the image, our robot would detect many octagons each frame, making the function useless. We also had trouble with the word STOP being written on the sign, which made it more difficult to find the outside of the sign. We thought that SIFT would be better for sign recognition and we pursued it for some time. However we found our background to be too noisy, and we were not able to find the proper thresholds for it. We switched back to contours given the better success rate and were able to improve on what we had already done so that only red octagons would be detected by the Neato. 

## What would you do to improve your project if you had more time?

Going forward, we would complicate the track.  Currently, the robot navigates by following a circular track.  Because the bot navigates by keeping the roadway centered within its field of view, it does not know how to handle single right-angle turns (as opposed to T intersections).  We could handle this by widening our field of view and mapping track characteristics to actions.

We would also build recognition for more signs.  As most signs are yellow diamonds, our color- and shape-detection methods used for the stop sign quickly becomes insufficient.  Using SIFT to compare template images could be one possible solution.

## Did you learn any interesting lessons for future robotic programming projects?
*These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.*

On workflow:
We initially decided to layer three different methods for sign recognition - color, shape, and text recognition - to create enough opportunities for our three-person team to experiment with computer vision.  There was enough work for each teammate, but we could perhaps have been more effective and have built more complex features had we consolidated early on.

### We named this Project Caribou because we gave animal names to all of the possible project ideas we had which combined a Neato and opencv. Because our project is similar to a self-driving car we used the animal CARibou. 

