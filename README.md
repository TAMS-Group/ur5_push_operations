# ur5_push_operations
This repository enables quasi-static push operations in a UR5 setup.
It contains packages for random push exploration, training of
forward push models, planning of push control paths and closed-loop
execution.

___Bringup___

Launch the full bringup:

```roslaunch tams_ur5_push_bringup bringup.launch```


Start the CLI:

```rosrun tams_ur5_push_execution cli```


Attach the pusher tool with help of the CLI:
  1. Enter ```maintenance``` to move the arm to the maintenance pose
  2. If necessary, open the gripper with command ```open```.
  3. Place the pusher tool inside the palm of the gripper
  4. Enter ```attach``` to add the pusher tool to the robot model
  5. Enter ```close``` to let the gripper grasp the tool
  
  
Now the setup is ready to perform pushes via the CLI or the plan execution interface.
  
___Exploration___
  
By default, the ```bringup.launch``` creates a directory 'push_results' in the home folder.
Executed pushes are recorded in CSV files within subfolders named after the date and time of the session.
Additionally images of the executed pushes can be recorded as well.
  
Random exploration sessions are run by entering the CLI command ```push nonstop```.


___Planning and Execution___

For planning the prediction package needs to be setup with the generated forward push models.
See the [prediction package](prediction) on how to do this.


Launch the planning pipeline:

```roslaunch tams_ur5_push_planning push_planning.launch```

In RViz the target object appears as interactive marker that can be moved on the table surface.
The marker represents the goal position for planning attempts.
A right click on the marker opens a context menu:
  - Plan: performs a single planning attempt and visualizes the result
  - Reset: removes the visualization and resets the marker position
  - Execute: performs an open-loop execution of the last plan
  - Run MPC: performs a Model-predictive Control attempt
  - Run multi-step MPC: performs a Model-predictive Control attempt while using as many steps of a plan as possible


  
