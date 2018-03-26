## Project: 3D Motion Planning
![Quad Image](./enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes python definitions in two files, `motion_planning.py` and the `planning_utility.py`.

The main of the program is called `plan_path()` resides in the `motion_planning.py`. It starts by reading the latitude and longitude of the home location in floating points from the external 2.5D `colliders.csv` map file.

The starter code reads in the obstacle map, runs a simple path from the center of the map (-`north_offset`, -`east_offset`), to (-`north_offset` - 10, -`east_offset` - 10). It then defines a grid for a particular altitude and safety margin around obstacles. It finally utilizes the A* search algorithm to find the optimum path from start to goal.

As can be seen from running the starter code without any modification, the drone flies in a zig-zag fashion from the A* seach algorithm. The zig-zag can be smoothed out by pruning the path using `collinearity` algorithm. The result is shown here.
![ZigZag effect](./zigzag.png)

After exploring the starter code, I am ready to build my own path planning algorithms that I have learnt from class. I will provide 2 solutions, one for the basic requirement, whihc is a `grid-based` A* path, and second a `graph-based` A* path planning algorithms. 

And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

First we need to set the global home position. The position is provided in the first line of the `colliders.csv` file. To read only the first line of the `colliders.csv` file in order to retrieve the home position, I created a module definition called `readrow0()`. The latitude, and longitude is then passed to the self.set_home_position() method to set the global home position.


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

Next, we need to know our current local position in NED format relative to global home. Once the global home position and the current global position is known, the NED current local position can be calculated from the global_to_local() method.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

The grid starting position can be then be provided from the current position relative to the map center.

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

Similarly, the grid_goal position can be calculated from any ( lat, lon ) within the map.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

In my basic `Grid_based_search` solution provided in a zip file, I modify the code in planning_util() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2). I also provided another solution which is based on graph, `Graph_based_search`.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

Once the path is calculated, I prune the path using the prune_path() method using collinearity test.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


