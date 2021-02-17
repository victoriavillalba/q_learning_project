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
  * plan: 
  * testing: Manually print out the q learning matrix and check whether the output action matches the optimal choice in the matrix.

* Determining the identities and locations of the three colored dumbbells
  * plan: Given a colored dumbbell (strictly red/green/blue), we utilize the camera’s 2d image to locate colored pixels on it, similar to the line follower example in class. We’ll utilize a small range of colors that should capture Red/Green/Blue values under shadows/lighting conditions. Once the given dumbbell color has been found, the turtle should rotate to keep the dumbbell in the centre of the camera. If the colored pixel is not found, we rotate the turtlebot by a set angle and repeat the process. 
  * testing: We can manually inspect this in Gazebo, to check if the turtle is indeed facing the given colored dumbbell. 

* Determining the identities and locations of the three numbered blocks
  * plan: Given a particular numbered block, we can utilize information from the camera to determine the identity of the block by feeding it to OpenCV for number recognition. This will also tell us the location of the particular block on the camera image, such that we can figure out the block’s direction relative to the camera. 
  * testing: 

* Picking up and putting down the dumbbells with the OpenMANIPULATOR arm
  * plan: calculate the angles of all the joints for grabbing the dumbbell and timing it right. Also the engles for when the dumbbell is lifted and put down. 
  * testing: run the code and see if the robot is able to gra, lift, and place down the dumbbell.

* Navigating to the appropriate locations to pick up and put down the dumbbells
  * plan: 
  * testing: 


## Timeline

* Feb 19: Q learning algorithm + convergence detection

* Feb 22: Determining identities/location of dumbbells + numbered blocks + moving to them + picking up

* Feb 26: Finishing writeup + record bags, gifs
