# Path_Planning_using_Dijkstra
Implementation of Dijkstra Algorithm for a Point Robot to Plan Path in a Obstacle Space

Code:
- Code consists of: 'Generating Map and it's Obstacles', 'Movement in 8 Directions', 'Using Search Algorithm and Back Tracking' and 'Visualizing the Tracked Path'
- The Search Algorithm used here is ' Dijkstra Algorithm' to reach end goal and obtain the path.
- The code has been delineated very clearly in the comments provided in the Code.

Dependencies:
- python 3.9 (works for any python 3 version)
- Python running IDE. (I used Spyder IDE to program the Code and Execute the Code)
- Libraries: numpy, matplotlib.pyplot, heapq, time

P.S. The visualization of the plotting is not smooth in spyder. Use Visual Studio etc.

Instructions to Run the Code:

To get the Output (without Visualization)
- Open the 'Dijkstra-pathplanning-Joseph-Katakam.py' file in any IDE. (I used Spyder)
- Run the Program
- In the Console, the program asks for the x and y coordinates of Start and Goal Node. Enter as prompted. Ex: Start: 50, 50; Goal: 150, 150
- The Output Plot with planned Path should be Visible.

To get the Output (with Visualization)
- Open the 'Dijkstra-pathplanning-Joseph-Katakam.py' file in any IDE. (I used Spyder)
- UnComment the line used for Visualization i.e. '309', which says- "plt.pause(0.000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001)"
- Run the Program
- In the Console, the program asks for the x and y coordinates of Start and Goal Node. Enter as prompted. Ex: Start: 80,110; Goal: 80,130
- The Visualization should be Displayed but takes a long duartion to move across a short space.

Understanding the Output Plot
- The Robot movable space is shown in Blue Color
- The Pixels in Maroon are the Obstacles.
- The Pixels in white is the 5mm Clearance Space.
- The explored path is marked by yellow color.
- The Planned Path is shown by Red Dotted Lines.
