# CS1567 Project 3

**Part 1: AprilTag Viewer**
A node that perform the following:
- Displays the video feed from the topic /image_raw using OpenCV
- For each detected ApirlTag, draws lines along edges of the tag
- For each detected ApirlTag, draws the tag ID in the middle of the tag

**Part 2: Search and Go**
With the Kobuki robot surrounded by AprilTags, the robot will turn to face a specific AprilTag and move toward it

**Part 3: Soccer Player Robot**
Two AprilTags should be used to represent the locations of left and right pole of a soccer goal. Another AprilTag should represent the location of the ball. The Kobuki robot will make a stationary turn to find both the poles and the ball, move to the appropriate position, and push the ball into the goal.

**Part 4: Follow the Breadcrumbs**
Given a series of AprilTags placed in a restricted environment, the Kobuki robot will move toward each AprilTag in order of increasing tag ID

