# Assignment 2 Report: Leader Election and Following  multi-agent systems

## Group 7

| Name                                         |   Email  |
|--------------------------------------------- |----------|
| Bernardo Tavares Monteiro Fernandes Portugal | au804038@uni.au.dk |
| Mohamed Hashem Abdou                         | au656408@uni.au.dk |
| Rafael Ferreira da Costa Silva Valquaresma   | au804039@uni.au.dk |
| Yu Jin                                       | au785458@uni.au.dk |

## Task Overview

**Basic Task:**  
Assume a MAS with 5 agents. Assume a leader agent (pre-assigned, and identifiable by the rest) and the rest
follower agents. Implement a nature-inspired mechanism (either ant or bird inspired) that enables the agents
to follow their leader, as well as keep some distance between each other (in a straight line or some other
formation). No direct communication is allowed between agents. Define the quality metrics and evaluate
your method

**Advanced Task:**  
Implement a leader election mechanism (e.g., bully). Have agents communicate to decide which position in
the fleet to take, use one of the mentioned coordination mechanisms. Scale the number of agents to, 10, 20
etc. Define the quality metrics and evaluate your method

---

## Solution Summary

### Basic Task: ACO Leader-Following

Our solution for the basic task is based on Ant Colony Optimization (ACO), an ant-inspired algorithm that uses indirect communication via environmental markers (pheromones).

- Implemented in:  
    - Basic Implementation
  [`assignment2/ACO.py`](/Ass2/MAS/assignment2/ACO.py)  
    - test with:
      [`assignment2/test1.py`](/Ass2/MAS/assignment2/test1.py)  

---


- **Mechanism (Indirect Communication):**
  - A global 2D grid simulates a **pheromone field**.
  - The **leader agent** moves along a predefined waypoint path, depositing a high concentration of pheromones (`PH_DEP_L`) to create a strong scent trail.
  - **Follower agents** are attracted to this trail and deposit a smaller amount of pheromones (`PH_DEP_F`) to reinforce the path.
  - Pheromones gradually **evaporate** (`PH_EVAP`) and **diffuse** (`PH_DIFF`), ensuring that the trail stays fresh and followers react to the leader's current path.

- **Follower Policy:**
  Followers choose their direction by balancing two main factors for multiple candidate headings:
  1.  **Pheromone Strength (`τ`):** The intensity of the scent trail in a given direction.
  2.  **Heuristic Value (`η`):** The desirability of a heading based on immediate sensor data. Our heuristic combines:
      - **Obstacle Clearance:** Distance to obstacles detected by the lidar sensor.
      - **Attraction to Leader:** Proximity to the leader's current position.
  
  A direction is probabilistically selected using the standard ACO formula: **`score = τ^α * η^β`**, which effectively merges trail-following with goal-seeking behavior.

- **Collision Avoidance:**
  All agents run a high-priority safety behavior that uses lidar data to calculate **Time-To-Collision (TTC)**. If an obstacle or another agent is too close, the agent will automatically slow down, stop, or execute an escape maneuver, ensuring agent and group safety.


#### **Running Example:**
<video width="320" height="240" controls>
  <source src="ex1.mp4" type="video/mp4">
</video>


--- 
## Quality Metrics & Results

To evaluate the performance of our ACO-based flocking system, we measure the following metrics, which are reported at the end of each simulation:

- **`mean_dist_to_leader`:** The average distance of all follower agents from the leader. A lower value indicates a tighter, more cohesive formation.
- **`mean_inter_member_dist`:** The average distance between any two agents in the swarm. This helps quantify the formation's spacing.
- **`follower_onpath_ratio`:** The percentage of time that followers spend within a defined band (`PATH_BAND = 0.5m`) around the leader's historical path. A higher value signifies better path fidelity.
- **`mean_pheromone_conc`:** The average pheromone intensity experienced by followers. This can indicate how well the followers are "on the trail."

### Experiment 1: Higher Beta than Alpha (This means agents prioritize moving towards the leader's *current* location (leader attraction) over following the historical path.)

- **Goal:** Evaluate the system behavior using the default parameters from `ACO.py`.

**System Parameters:**

| Parameter          | Value  | Description                                          |
|--------------------|--------|------------------------------------------------------|
| `ALPHA`            | `2.0`  | Weight of the pheromone trail (`τ`)                  |
| `BETA`             | `1.0`  | Weight of the heuristic (`η`)                        |
| `ETA_W_LEADER`     | `0.8`  | Weight for leader attraction in the heuristic        |
| `PH_EVAP`          | `0.99` | Pheromone retention rate per step (1% evaporation)   |
| `VN` (Follower Speed) | `0.75` | Nominal forward speed scale for follower agents      |

**Observed Results (2000 steps):**

| Metric                     | Value     |
|----------------------------|-----------|
| `mean_dist_to_leader`      | `4.002`   |
| `mean_inter_member_dist`   | `1.742`   |
| `follower_onpath_ratio`    | `0.0`     |
| `mean_pheromone_conc`      | `20.188`  |

**Commentary on Results:**

The results from our baseline test are a direct reflection of the chosen system parameters:

1.  **Loose Formation (`mean_dist_to_leader`: 4.002):** The average distance to the leader is very high. This is primarily because the followers' nominal speed (`VN = 0.75`) is significantly lower than the leader's speed (`1.0`), causing them to fall behind consistently.

2.  **Heuristic Dominance:** The parameters `ALPHA = 1.0` and `BETA = 2.5` place a much stronger emphasis on the heuristic component (`η`) than on the pheromone trail (`τ`). This means agents prioritize moving towards the leader's *current* location (leader attraction) over following the historical path.

3.  **Complete Failure of Path Following (`follower_onpath_ratio`: 0.0):** This metric is the most telling consequence of the parameter settings. Because the heuristic (`BETA`) is dominant and followers are slow, they are constantly trying to "cut corners" towards the distant leader instead of following the trail. They never manage to get on the path, so the pheromone component of their decision-making becomes completely irrelevant. The system is not behaving like an ant colony but rather like a simple "move-towards-leader" system.

### Experiment 2: Balanced Trail-Following (High Alpha, Moderate Beta)

- **Goal:** Evaluate system performance with parameters balanced to favor trail-following (`ALPHA`) while providing a moderate heuristic (`BETA`) to help lost agents recover.

**System Parameters:**

| Parameter          | Value  | Description                                          |
|--------------------|--------|------------------------------------------------------|
| `ALPHA`            | `2.5`  | **High** weight for the pheromone trail (`τ`)        |
| `BETA`             | `1.5`  | **Moderate** weight for the heuristic (`η`)          |
| `ETA_W_LEADER`     | `1.9`  | **Moderate** weight for leader attraction            |
| `PH_EVAP`          | `0.97` | Pheromone retention rate per step (1% evaporation)   |
| `VN` (Follower Speed) | `0.95` | Nominal forward speed scale for follower agents      |

**Observed Results (2000 steps):**

| Metric                     | Value     |
|----------------------------|-----------|
| `mean_dist_to_leader`      | `4.700`   |
| `mean_inter_member_dist`   | `2.415`   |
| `follower_onpath_ratio`    | `0.0`     |
| `mean_pheromone_conc`      | `22.609`  |

**Commentary on Results:**

This experiment aimed to fix the issues from the previous tests by creating a more balanced agent policy. The hypothesis was that a strong pheromone weight (`ALPHA`) would ensure trail-following, while a moderate heuristic (`BETA`) would act as a "leash" to guide lost agents back to the trail.


**Analysis of system0:**
The continued failure, even with theoretically better parameters, points to a fundamental initialization or sensitivity problem in the system.
- **Initial State is Critical:** At the beginning of the simulation, the followers start "off the trail." The moderate heuristic was intended to guide them onto it. The fact that they never succeeded suggests that by the time they get close, the original pheromone trail has already evaporated too much to be detected strongly.
- **System:** This demonstrates the fragility of a purely pheromone-based system. It relies on agents staying in continuous contact with the trail. If contact is ever broken (due to speed differences, obstacles, or poor initial position), the agent may not have a robust enough mechanism to recover and find the trail again.


**Overall Conclusion from Experiments:**
Across all tested parameter configurations, the agents have consistently failed to follow the leader's path as intended by the ACO model. This is not a failure of the algorithm's theory but an illustration of its practical challenges. The system is highly sensitive to initial conditions and the interplay between agent speed, evaporation rates, and the strength of the recovery heuristic. For this system to work, the agents must be able to acquire the trail very early in the simulation and be fast enough to never lose it.


## Code

Available at https://github.com/Valquaresma03/MAS/tree/main/assignment2
