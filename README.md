# q_learning_project

__Team Members: Yves Shum and Victoria Villalba__

## Running 

### Q Learning 
- Start `roscore`
- Run Gazebo launch file `roslaunch q_learning_project turtlebot3_intro_robo_manipulation.launch`
- Run Phantom movement file `rosrun q_learning_project phantom_robot_movement.py` 
- Run Q learning module `rosrun q_learning_project q_learning.py`

### Actual actions 

- Start `roscore`
- Run Gazebo launch file `roslaunch q_learning_project turtlebot3_intro_robo_manipulation.launch`
- Run Moveit config launch `turtlebot3_manipulation_moveit_config move_group.launch`
- Run perception_manipulation module `rosrun q_learning_project perception_manipulation.py`
- Run controller node once perception_manipulation outputs ready. `rosrun q_learning_project controller_node.py`
- Sit back and watch

## Writeup

- Objectives
  - The goal of the project is to be able to train a Q-Learning model to learn to place certain dumbbell colors near the block that it's associated with. 
  - The project involved utilizing Image and LaserScan data to understand where the turtle is, and perceiving where the blocks/dumbbells are 
  - It also involved motor control of picking up dumbbells and dropping them off

- High level description
  - We utilized the Q-Learning algorithm to build up a Q-Matrix that described the optimal action for each possible state. This algorithm worked by selecting random valid actions depending on a particular robot state, carrinyg out the action, and receiving a reward given by the environment depending on whether the action was correct. We first built 0-initialized QMatrix, then constructed an action matrix detailing what actions are valid for what state. Then we carried out the Q-Learning loop to select a random valid action depending on the current state, carrying out the action with the `phantom_robot_movement` module, then receiving a reward from `reset_world.py`. With this reward we update the `QMatrix` accordingly using the algorithm given in class, with `alpha=1` and `gamma=0.2`. Once the matrix converges, the turtle has successfully learnt which dumbbell goes with which block.

- Q-learning algorithm description
  - Selecting and executing actions for the robot (or phantom robot) to take
    - We first initialized an action matrix in `q_learning.initialize_action_matrix`, that calculates all valid actions to take given a particular state. Valid actions to take basically follow the rule of 1) Not moving dumbbells to origin 2) Not moving dumbbells to a block where a dumbbell already exists 3) No moving dumbbell to the same place.
    - Selecting actions is done in the function `q_learning.sample_action`, which checks the current state and indexes into the action matrix, selecting a valid action at random
  - Updating the Q-matrix
    - The Q-matrix is updated whenever a reward is received. This is primarily handled through the `q_learning.reward_callback` function
    - Based on the received reward and the action that we took, we update the q matrix via the formula given in class, taking into account the maximum reward of the next state
    - We utilized `alpha=1` and `gamma=0.2` in our learning
    - We also had to catch a bug where sometimes we would receive a reward when the world resets
  - Determining when to stop iterating through the Q-learning algorithm
    - Every time we updated a value in our QMatrix, we added the value's change to a running window of length 200, meaning that it contains the last 200 changes made to the QMatrix. This is handled inside `q_learning.reward_callback`
    - After modifying the running window, we determine that the QMatrix converged if all of the last 200 changes did not produce meaningful change (i.e. changing 100 to 100, 0 to 0, 4 to 4 etc.). This is handled in `q_learning.check_matrix_converge`
    - Once the QMatrix converge, we stop running the Q-learning algorithm and output the QMatrix to a dump file. 
  - Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot
    - Given a particular state, we were able to choose the best action by indexing into the QMatrix with the current state, and selecting the action that corresponded to the maximum reward. 
    - If there were multiple maximum rewards, we simply picked the first one
    - This is handled in `controller_node.selectActionFromState`
- Robot perception description
  - Identifying the locations and identities of each of the colored dumbbells
    - TODO:
  - Identifying the locations and identities of each of the numbered blocks
    - Identity of the numbered blocks was identified via passing Image data to `keras-ocr`. We explicitly looked for the characters `1` or `l`, `2`, `3`. This is handled in `perception_manipulation.move_to_block`
    - No further online sources were used other than the keras documentation linked by the project outline 
    - Identifying the location of numbered blocks were done given the prediction groups outputted by `kreas-ocr`. We took the center point coordinates of the prediction box to compare with the center of the screen to obtain the direction of the desired numbered block. This is handled also by `perception_manipulation.move_to_block`
- Robot manipulation and movement 
  - Moving to the right spot in order to pick up a dumbbell
    - TODO: 
  - Picking up the dumbbell
    - TODO: 
  - Moving to the desired destination (numbered block) with the dumbbell
    - Given the prediction groups outputted by `keras-ocr`, we used proportional control on the center of the prediction box compared to the center of the Image to determine the angular velocity of the turtle. 
    - If no prediction was found, we had to robot run on search mode, rotating slowly on with z angular velocity. 
    - Our robot stopped every few seconds to allow for `keras-ocr` to slowly parse through the image so that we can instruct our tutlebot accurately on which direction to move towards 
    - Once we were close enough (~1m) away from the obstacle directly in front, we stopped using information from the Camera since it was too close to recognize numbers anyways. Instead, we used LaserScan information to move as close as possible to the block.
    - This is handled in `perception_manipulation.move_to_block`
  - Putting the dumbbell back down at the desired destination
    - Once the turtle is directly in front of the desire destination, we.. TODO: 
- Challenges
  - World reset reward bug 
    - There was a race condition that led to another reward being published whenever the world reset, on top of the reward for reaching the final state (where a dumbbell is in front of each block). This messed up the control logic for our QLearning algorithm, as it triggered the `q_learning.reward_callback` more than it should have (messing up the algorithm's believed state).
    - We fixed this by catching for the bogus reward, looking at the timestamp that it was published. The bogus reward always came within 0.01 seconds of the last reward, where we caught and ignored this case in `q_learning.reward_callback`.
  - OCR performance and manipulation delay 
    - Running OCR on a camera Image loop continuously led to a lot of problems with instructing the turtle to move accurately. The main issue is that by the time we've finished processing 1 image, the turtle has already moved for a certain amount of time. Whatever instruction we gave to the robot became obsolete and inaccurate.
    - We overcame this issue by stopping the turtlebot whenever we had to do OCR on the image output. This led to the robot exhibitng clunky movements and it started and stopped moving. 
  - TODO: dumbbell manipulation
- Future work
  - If we had more time, we could have implemented a Odometry-based solution, where we would first scout and map out the pose of the numbered blocks, keeping track of the 3 points in front of each block. This would avoid having to repeatedly run OCR when instructing the turtle to move towards the desired block 
  - We would also fine-tune the q-learning parameters such as `alpha` and `gamma` to achieve faster matrix convergence 
  - TODO: 
- Takeaways
  - TODO:

## Gif 
- TODO: once everything is done

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
