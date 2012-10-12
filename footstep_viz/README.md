Installation:

You'll need to modify one of PCL's header files first (FootstepVisualizer 
subclasses PCLVisualizer and needs access to private fields). 

Open the pcl/visualization/pcl_visualizer.h file. (on Ubuntu it's under
/usr/include/pcl-1.6/pcl/visualization/pcl_visualizer.h).

Delete the line that goes "private:."

Building:

mkdir build && cd build; cmake ..; make

Running:
./footstep_viz ../table_scene_lms400.pcd -d -f -s -n

Using:
Check main.cpp

Output:

The output should look something like: http://imgur.com/nYjge

Hopefully our algorithm won't plant steps on the sides of tables!
