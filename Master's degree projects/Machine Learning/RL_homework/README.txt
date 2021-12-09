Pacman starts from a central position in a 10x10 grid world, where there are 5 ghosts and 35 walls.
The reward of every quare in the grid is 1, while the reward of ghosts is -100 and the reward in
the initial position is 0. When Pacman eats from a square, the reward of that square becomes 0,
so that when Pacman passes from there again, he doesn't get any reward. Initially, the agent has 
3 lives, which means that he will die only when he hits a ghost for the third time. 
So, if the agent finishes its lives or reaches the goal of eating everything in the world, the 
experiment ends. If the agent hits a ghost, but he still has at least a life, it goes back to the 
initial position, and starts performing random actions again. When the agent hits a ghost but does 
not die, the reward -100 gets summed to the total reward, so that the agent can keep it into acconunt 
while learning.