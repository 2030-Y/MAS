# Assignment 1 Report: RL-Based Circle Following in Multi-Agent System

## Group 7

| Name                                         |   Email  |
|--------------------------------------------- |----------|
| Bernardo Tavares Monteiro Fernandes Portugal | au804038@uni.au.dk |
| Mohamed Hashem Abdou                         | au656408@uni.au.dk |
| Rafael Ferreira da Costa Silva Valquaresma   | au804039@uni.au.dk |
| Yu Jin                                       | au785458@uni.au.dk |

## Task Overview

**Basic Task:**  
Implement an RL to teach your robot how to move in a circle (around some defined point in the 2D space).
Instantiate a MAS of 5-10 robots, where each robot runs the circle following behaviour, and measure the
quality of your solution. You can determine yourself the quality metrics to consider

**Advanced Task:**  
Use the subsumption architecture with two layers, obstacle avoidance, and circle following. Evaluate the
performance of one agent, and continue by scaling the system up to 5, 10, 20 etc. Define quality metrics at
the agent level and group level. Evaluate your solution

---

## Solution Summary

### RL Circle Following

- Implemented in:  
    - Basic Implementation
  [`assignment1/custom_behavior_methods_1.py`](/irsim/MAS/assignment1/custom_behavior_methods_1.py)  

    - Advanced Implementation
  [`assignment1/custom_behavior_methods_2.py`](/irsim/MAS/assignment1/custom_behavior_methods_2.py)

- **State Definition:**  
  The agent’s state is defined by its position, orientation, and distance/angular error relative to the target circle, center at [5,5] and radius 3


- **Reward Function:**  
  Agents receive higher rewards for staying close to the circle; penalties are applied for collisions.

- **RL Update:**  
  Value function is updated using experience, with transitions and rewards stored and periodically re-evaluated.

- **Policy:**  
  Agents adjust their linear and angular velocity based on their deviation from the circle, learning to follow the trajectory.

### Multi-Agent System (MAS)

- Multiple agents are instantiated and their interactions are tracked.  
- Peer metrics such as minimum and mean separation are calculated to monitor group safety and coordination.

### Subsumption Architecture (Advanced)

- **Layers:**  

  1. Obstacle Avoidance (priority)
  2. Circle Following (default behaviour)

- When obstacles are detected or agents are too close, obstacle avoidance overrides circle following.

- Implemented using the `subsumption_nav` registered behaviour.

---

## Quality Metrics

Reported for both individual agents and the agent group:

- **Agent Level:**
  - Mean radial error (distance from circle)
  - Mean angular error (alignment to tangent)
  - Time on target ratio (proportion of steps near circle)

- **Group Level:**
  - Avoidance ratio (steps spent avoiding obstacles)
  - Minimum separation (closest agents)
  - Mean separation (average distance between agents)

Metrics are calculated and printed via the `metrics_p()` function.

---

## Example Code References

- RL update and reward logic:
  ```python
  # assignment1/custom_behavior_methods_1.py
  def RL_passive(ego_object, objects=None, *args, **kwargs):
      # ... RL logic
  ```

- Subsumption navigation:
  ```python
  # assignment1/custom_behavior_methods_2.py
  @register_behavior("diff", "rl_passive_2")
  def subsumption_nav(ego_object, objects=None, *args, **kwargs):
      # ... combines avoidance and circle following
  ```

- Metrics computation:
  ```python
  def metrics_p():
      # ... collects mean error, time on target, separation, etc.
  ```

---

## Results

### ADP Progress (Test 1)

| Steps | Visited States | Total States | AvgV     | Mean [dist] |
|-------|:--------------:|:------------:|---------:|---------:|
| 200   | 5              | 120          | -134.407 | 1.148    |
| 400   | 7              | 120          | -109.927 | 0.795    |
| 600   | 7              | 120          | -106.734 | 0.640    |
| 800   | 7              | 120          | -105.834 | 0.627    |
| 1000  | 7              | 120          | -105.589 | 0.637    |
| 1200  | 7              | 120          | -105.505 | 0.643    |
| 1400  | 7              | 120          | -105.460 | 0.645    |
| 1600  | 7              | 120          | -105.428 | 0.645    |
| 1800  | 7              | 120          | -105.404 | 0.645    |
| 2000  | 7              | 120          | -105.385 | 0.645    |
| 2200  | 7              | 120          | -105.369 | 0.645    |
| 2400  | 7              | 120          | -105.357 | 0.645    |
| 2600  | 7              | 120          | -105.347 | 0.645    |
| 2800  | 7              | 120          | -105.338 | 0.645    |
| 3000  | 7              | 120          | -105.330 | 0.645    |
| 3200  | 7              | 120          | -105.324 | 0.645    |
| 3400  | 7              | 120          | -105.318 | 0.645    |
| 3600  | 7              | 120          | -105.313 | 0.645    |
| 3800  | 7              | 120          | -105.309 | 0.645    |
| 4000  | 7              | 120          | -105.305 | 0.645    |
| 4200  | 7              | 120          | -105.302 | 0.645    |
| 4400  | 7              | 120          | -105.298 | 0.645    |
| 4600  | 7              | 120          | -105.295 | 0.645    |
| 4800  | 7              | 120          | -105.293 | 0.645    |
| 5000  | 7              | 120          | -105.290 | 0.645    |

#### Final Metrics

| Mean Radial Error | Mean Angular Error | Time On Target Ratio |
|-------------------|-------------------|---------------------|
| 0.682             | 0.072             | 0.944               |

#### Commentary on Results (Test 1)
- **ADP Progress**
    - **Visited States:** The number of visited states quickly rises from 5 to 7 and then plateaus, meaning the policy is focusing on a small, relevant subset of the state space associated with successful circle following.
    - **AvgV (Average State Value):** The average value function (avgV) starts very negative and becomes less negative over time, indicating the agent is learning to avoid poor states and maximize long-term reward.
    - **Mean |dist|:** The mean distance to the target trajectory steadily decreases, showing effective learning. After about 600 steps, the agent consistently maintains a low distance to the circle.

- **Final Metrics**
    - **Mean Radial Error (≈ 0.68):** This low value shows the agent is able to closely follow the circular trajectory most of the time.
    - **Mean Angular Error (≈ 0.07):** The agent’s heading is well-aligned with the tangent of the circle, demonstrating precise control.
    - **Time On Target Ratio (≈ 0.94):** The agent spends 94% of the time on or near the target circle, a strong indication of successful policy learning and stable behavior.

The agent quickly learned to stay close to the circle and maintain the correct heading.
High time-on-target ratio and low error metrics confirm robust and reliable circle-following behavior.
The RL agent demonstrates effective and stable circle-following, with quick learning and sustained performance. The metrics reflect precise control and a policy that is well-adapted to the task, supporting the success of our implementation.

### Metrics Summary (Test 2)

| Mean Radial Error | Mean Angular Error | Time On Target Ratio | Avoid Ratio (%) | Min Separation | Mean Separation | Steps |
|-------------------|-------------------|---------------------|-----------------|----------------|-----------------|-------|
| 1.80              | 1.34              | 0.00                | 65.67           | 0.68           | 0.73            | 150   |

### Commentary on Results (Test 2)

- **Mean radial error (1.80) and mean angular error (1.34) are high**, indicating agents often deviated from the target circle and were frequently misaligned.
- **Time on target ratio is 0.00**, showing agents were almost never able to follow the intended trajectory.
- **Avoid ratio of 65.67%** suggests agents spent most of the simulation actively avoiding obstacles and the other agents as priority rather than performing the desired circle-following behavior.
- **Minimum and mean separation (0.68 and 0.73)** are relatively low, pointing to frequent close encounters between agents, likely due to crowding or challenging navigation.
- **Total steps (150) are much lower than in other tests,** indicating a shorter or more challenging simulation scenario that limited learning opportunities.

**Summary:**  
Test 2 demonstrates a very challenging environment with obstacle avoidance dominating agent behavior. The metrics reflect difficulty in maintaining the desired trajectory and safe distances, highlighting the impact of scenario complexity and agent interactions in multi-agent reinforcement learning.

---

## Conclusion

The results from both basic and advanced tests highlight the strengths and limitations of our multi-agent reinforcement learning solution for circle following.

In the **basic scenario**, agents rapidly learned to follow the circle with high precision and stability, as shown by the low mean radial and angular errors and a high time-on-target ratio. This demonstrates that the RL approach is highly effective for coordinated control when the environment is less challenging and agent interactions are minimal.

In the **advanced scenario**, the presence of obstacles and increased agent interactions made the environment significantly more challenging. The metrics from Test 2 reveal that agents spent most of their time avoiding obstacles and other agents, resulting in high mean errors and a time-on-target ratio of zero. The frequent close encounters and low separation values suggest that navigation and coordination become much harder as complexity rises. 

Overall, the experiments show that while our RL and subsumption architecture can achieve robust and reliable circle-following behavior in controlled settings, performance declines in highly dynamic or crowded environments where obstacle avoidance must take priority. These findings reinforce the importance of environment design, agent coordination strategies, and adaptive behaviors for scalable multi-agent learning. Future work should focus on improving obstacle avoidance integration and enabling more effective learning in dense, complex scenarios.

## Code

Available at https://github.com/Valquaresma03/MAS/tree/main

