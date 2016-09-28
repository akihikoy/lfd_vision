lfd_vision
==================
All vision stuff for PR2, Baxter, whole-body vision experiments.  Because of my laziness, I put everything in this repository.

lfd_vision and other robot controllers communicate over ROS (Robot Operating System) architecture.

ROS:
http://wiki.ros.org/

Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Acknowledgment
==================
Some of programs use a ray tracing library made by Don Cross (the files are put in include/lfd_vision/raytrace/doncross/).
http://www.cosinekitty.com/raytrace/

Some of programs use a PNG library LodePNG made by Lode Vandevenne (the files are put in include/lfd_vision/raytrace/lodepng).


Requirements
==================
Install following packages before building lfd_vision.

Binaries are available (you can use **apt-get**):
- ROS core system, rospy, roscpp, std_msgs, std_srvs, ...
- OpenCV
- PCL (Point Cloud Library)

Need to build from source (these are using **Catkin**):
- http://wiki.ros.org/ar_track_alvar
- http://wiki.ros.org/ar_track_alvar_msgs


Build
==================
The repository directory should be in ROS workspace (e.g. ~/ros_ws/).
Build lfd_vision with rosmake.

```
$ rosmake lfd_vision
```

After rosmake, you will find some executables in bin/ directory.
There are some build directories made by ROS.


Programs
==================
Programs can be executed by "launch" files stored in launch/ directory.

```
$ roslaunch lfd_vision LAUNCH_FILE.launch
```

ar_track_ext_xtion.launch:
Just launches ar_track_alvar (AR marker tracker) which is a pose estimator of AR tags.
We use an Xtion camera.

color_detector.launch:
Flow detection, materials detection, and container detection with colors.
bin/color_detector_node is executed.
We use a (USB) RGB camera.

```
Right click: Pause/Resume
Space:  Pause/Resume
Shift+Left click: register the clicked color for the detector.
Shift+Right click: clear the registered colors.
s: save the registered colors into a file.
l: load colors from a file.
1,2: change the active color filter to 1 or 2.
0: change the active color filter to one for flow detection.
], [: rotate the image.
m: change the display mode.
```

color_detector2.launch:
Execute two bin/color_detector_node.
We use two (USB) RGB camera.

sentis_tof_m100_s.launch:
Launch Sentis depth sensor M100.

rt_pose_estimator_xtion.launch:
Object pose estimator with a rendering-based approach.
Ray-tracing is used to render the object.
We use Xtion camera.

rt_pose_estimator_m100.launch:
Similar to rt_pose_estimator_xtion.launch but we use Sentis depth sensor M100.
sentis_tof_m100_s.launch should be executed beforehand.


Troubles
==================
Send e-mails to the author.  I accept only from my students.
