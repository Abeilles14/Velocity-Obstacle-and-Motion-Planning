# Velocity-Obstacle-and-Motion-Planning
A Collision Avoidance Framework implemented for a dual arm Pick and Place robot task simulation, using Velocity Obstacles and RRTStar Motion Planner.  
  
UBC Electrical Engineering  
_ELEC 499: Bachelor's Thesis_  


This thesis focuses on the development of an integrated task and motion planning (TMP) strategy for a multilateral automated model, providing a flexible task coordination approach in a dynamic environment. The sampling based motion planner RRT* is implemented to generate a collision-free path for each PSM moving towards a point of interest. The position and range of each individual arm is tracked in order to avoid possible collisions using a Velocity Obstacle approach.  
The main idea of this approach is to compute collision-avoiding velocities with respect to other robots, such that each robot may continue on their path without changing their initial trajectory. The objectives of this work concern identifying and handling collisions with the static environment obstacles and potential collisions with PSMs dynamically without impeding the parallelism aspect of the robotic arm movements.  

Demo Link: https://youtu.be/wWmJCkLgELY
