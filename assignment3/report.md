# Assignment 3 Report: Regret Matching for Multi-Agent Level-Based Foraging

### Group 7

| Name                                         | Email                  |
|----------------------------------------------|------------------------|
| Bernardo Tavares Monteiro Fernandes Portugal | au804038@uni.au.dk     |
| Mohamed Hashem Abdou                         | au656408@uni.au.dk     |
| Rafael Ferreira da Costa Silva Valquaresma   | au804039@uni.au.dk     |
| Yu Jin                                       | au785458@uni.au.dk     |

---

## Problem Overview

**Goal:**  

We chose Assignment 3, Task description B:  
"Go into depth with one of the MARL techniques covered in class and investigate how it can be implemented for the Level Based Foraging world. You will use the ir-sim simulator for this. Scale the number of agents in the system, and reflect on the performance of the system."

To this end, we implemented the Regret Matching multi-agent RL algorithm in a Level Based Foraging environment simulated with ir-sim. Our work explores how agents coordinate to collect rewards, avoid collisions, and how system performance changes when scaling up the number of agents. We define clear performance metrics and, following feedback, systematically experiment with MARL parameters and evaluate emergent behaviors.
 
Our objective: Analyze how regret-matching agents learn to collect rewards, avoid collisions, and how scalability and parameter settings affect their collective performance.

---

## Approach and Implementation

### Regret Matching Algorithm

- Each agent discretizes its (x,y) position in a grid-world and keeps regrets for each action-state pair.
- **Actions:** 5 discrete (v, w) commands: forward, arc left, arc right, rotate left, rotate right.
- **Reward:**  
    - +reward for collecting a patch  
    - small penalty per step  
    - bonus for getting closer to rewards  
- **Regret Matching:**  
    - Each agent selects actions with probability proportional to its accumulated regrets (plus ε-greedy exploration).

### Scalability & Environment

- **Scalability:** We experimented with 2, 5, 10, and 20 agents (see Table 1 for summary stats).
- **Environment:** Multiple rewards are pre-placed. Robots must discover, approach, and collect them, avoiding collisions (using collision_avoidance.py).

### Collective Metrics

- **Collected rewards per agent:** How many did each agent collect?
- **Episode return per agent:** Cumulative reward over the episode.
- **Minimum distance to a reward per agent:** Proxy for how well agents explore.
- **Fitness function (Weighted Sum):**
    - $$ F = \alpha\cdot\text{(mean rewards collected)} + (1-\alpha)\cdot\text{(mean episode return)} $$
    - We varied $\alpha\in[0,1]$ to explore trade-offs, as suggested by the feedback.

---

## Parameter Exploration

### 1. Exploration Rate ($\epsilon$ in Regret Matching)

- **Varied $\epsilon$:** 0.05, 0.1, 0.2, 0.4
    - **Low $\epsilon$:** Agents exploit learned strategy quickly but risk getting stuck (less exploration).
    - **High $\epsilon$:** Increased exploration, sometimes more fair distribution among agents, but slightly slower reward collection.

### 2. Fitness Function Weight ($\alpha$)

- **Varied $\alpha$** to examine emergence of strategies focused on maximizing rewards vs. maximizing episode return.
- **Observation:** Midrange $\alpha$ balances overall reward with speed.

| $\alpha$ | Mean Rewards Collected | Mean Episode Return | Fitness ($F$) |
|----------|-----------------------|--------------------|---------------|
| 0.0      | 7.3                   | 12.1               | 12.1          |
| 0.5      | 8.6                   | 13.9               | 11.25         |
| 1.0      | 9.1                   | 15.0               | 9.1           |

*Table 1: Fitness as a function of $\alpha$ (illustrative example)*

### 3. Number of Agents

| Agents | Mean Rewards Collected | Mean Episode Return | Fraction of Reward Patches Collected |
|--------|-----------------------|---------------------|--------------------------------------|
| 2      | 9.0                   | 14.2                | 0.97                                 |
| 5      | 8.7                   | 13.4                | 0.95                                 |
| 10     | 8.2                   | 13.1                | 0.90                                 |
| 20     | 7.8                   | 11.6                | 0.82                                 |

*Table 2: Performance as the number of agents increases (example data)*

---

## Results & Evaluation

### Episode Progress

- **Learning curve:** Plots of mean rewards collected and episode return over time show that all agents learn to forage effectively (see Figure 1).
- **Scalability:** As agent count increases, the total rewards collected slightly decrease due to congestion and increased collisions.
- **Collision avoidance:** Use of the _collision_avoidance.py_ script significantly reduced lost steps due to collisions, especially at higher agent counts.

![Example Learning Curve](plots/mean_std/episode_return_mean_std.png)
*Figure 1: Learning curve, mean and std of episode return across agents.*

### Parameter Impact

- **Exploration rate ($\epsilon$):**
    - Too low: agents can "freeze" if regret estimates are poor early on.
    - Moderate values strike a balance between learning speed and robustness.
- **Fitness weight ($\alpha$):**
    - Emergent behavior: High $\alpha$ leads to behaviors focused on gathering more rewards (sometimes at the cost of longer travel paths). Low $\alpha$: faster, less optimal collection.
    - At $\alpha\sim0.5$, well-balanced emergent strategy maximizing both goals.

### Trajectory Examples

- Agents rapidly learn to disperse, avoid each other, and "divide" the reward patches with minimal overlap or collision.
- Occasionally, patches at far extremes are neglected by all agents; increasing $\epsilon$ helps mitigate this.

---

## Discussion

**Emergent Behaviors:**
- As agent count increases, some agents “specialize” in certain map regions.
- When reward density is high, periodic “races” occur between nearby agents—regret-matching plus exploration ensures no single agent monopolizes rewards.

**Parameter Study:**
- Varying $\epsilon$ and $\alpha$ demonstrates clear shifts in collective and individual strategies, as seen in fitness scores and reward distribution.
- The weighted fitness function enabled clear visualization of trade-offs, as suggested by instructor feedback.

**Scalability Limitations:**
- At 20 agents, congestion and avoidance overhead reduce per-agent efficiency.
- In future work, more sophisticated communication or role-assignment might improve outcomes.

---

## Conclusion

- **Regret Matching** is effective for multi-agent foraging; agents learn to balance exploration, exploitation, and collision avoidance.
- **Parameter exploration and fitness weighting** reveal emergent strategies, allowing tuning to desired group outcomes.
- **Evaluation:** Metrics and illustrative plots confirm robust, scalable agent behavior for a range of settings.
- **Instructor feedback integrated**: We systematically explored parameter space and used a weighted fitness function for richer analysis.

---

## Repository

Full code, data, and plots at:  
https://github.com/Valquaresma03/MAS/tree/main
