# q_learning_project

__Team Members: Yves Shum and Victoria Villalba__

## Implementation Plan (initial)

* Executing the Q-learning algorithm
  * plan: implement the algorthm from class meeting 10.
  * testing: print values and check rewards.

* Determining when the Q-matrix has converged
  * plan: stop the robot as soon as it has converged.
  * testing: check if the robot stops, repeat a few times.

* How to determine which actions the robot should take to maximize expected reward after convergence
  * plan: Given a converged Q matrix, we simply index into the matrix rows to find the action (column) with the highest reward given a particular state.
  * testing: Manually print out the q learning matrix and check whether the output action matches the optimal choice in the matrix.

* Determining the identities and locations of the three colored dumbbells
  * plan: Given a colored dumbbell (strictly red/green/blue), we utilize the camera’s 2d image to locate colored pixels on it, similar to the line follower example in class. We’ll utilize a small range of colors that should capture Red/Green/Blue values under shadows/lighting conditions. Once the given dumbbell color has been found, the turtle should rotate to keep the dumbbell in the centre of the camera. If the colored pixel is not found, we rotate the turtlebot by a set angle and repeat the process. 
  * testing: We can manually inspect this in Gazebo, to check if the turtle is indeed facing the given colored dumbbell. 

* Determining the identities and locations of the three numbered blocks
  * plan: Given a particular numbered block, we can utilize information from the camera to determine the identity of the block by feeding it to OpenCV for number recognition. This will also tell us the location of the particular block on the camera image, such that we can figure out the block’s direction relative to the camera and rotate to keep the block near the center. We can search for blocks either by looking for white colored blocks and rotating the camera, or utilizing the scan to look for straight edges that make up a block.
  * testing: Given a block, we can have the turtle look for the block with the corresponding number, then check if it ends up facing the block in Gazebo.

* Picking up and putting down the dumbbells with the OpenMANIPULATOR arm
  * plan: calculate the angles of all the joints for grabbing the dumbbell and timing it right. Also the engles for when the dumbbell is lifted and put down. 
  * testing: run the code and see if the robot is able to gra, lift, and place down the dumbbell.

* Navigating to the appropriate locations to pick up and put down the dumbbells
  * plan: Once we’ve determined the location of the dumbbells, we navigate towards it by keeping the dumbbell at the centre of the camera view and moving towards it. We then stop the turtle’s linear velocity once we’re close enough. Proximity can be calculated by the /scan information, where we consider the distance directly in front of the turtle. 
  * testing: Given a specific dumbbell, we run our function to navigate to the dumbbell and watch it do its thing in Gazebo. 


## Timeline

* Feb 19: Q learning algorithm + convergence detection

* Feb 22: Determining identities/location of dumbbells + numbered blocks + moving to them + picking up

* Feb 26: Finishing writeup + record bags, gifs
