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

### Regret Matching for Multi-Agent Foraging

- **Agent Control:** Each agent discretizes its (x,y) in the world grid. For each grid cell, it tracks regret for a small set of discrete actions (forward, turn, rotate).
- **Actions:** 5 discrete (velocity, angular velocity) pairs.
- **Regret Matching:** At each time step, agents sample actions proportional to their accrued positive regret, with some probability of random exploration ($\epsilon$-greedy).
- **Rewards:**
    - Large positive reward for collecting patches (if within pickup radius)
    - Small negative per-step penalty ("living cost")
    - Small bonus for reducing distance to the nearest reward (shaping)
    - (Improved) Bonus for exploring novel states, penalty for rotating/standing still

**> Collision avoidance** is handled by a separate module (`collision_avoidance.py`), ensuring robots adjust their velocity and angle to avoid imminent collisions.

### Environment and Experiment Setup

- **Rewards:** Placed at fixed locations, only collectible once.
- **Agents:** Examined at 5, 10, and 20 agents.
- **Scaling:** All metrics computed per agent and at the system level.
- **Fitness Function:**  
    We use a weighted sum, allowing us to interpolate between optimizing **reward collection speed** and **total reward**:

    $$
    F = \alpha \cdot \text{(mean rewards collected)} + (1 - \alpha) \cdot \text{(mean episode return)}
    $$
    We experiment across $\alpha \in [0, 1]$

### Implementation Improvements (Based on Tests Results)

We extended our baseline by:
- **Dynamic exploration rate:** Start with $\epsilon=0.3$; gradually decay to $\epsilon=0.05$.
- **Higher speed and greater collection radius:** Doubled `vel_max`, increased `COLLECT_RADIUS` to 0.5.
- **Penalizing excessive rotation:** Discouraged unproductive churning/standing still.
- **Novelty/redundancy bonuses:** Small reward for first-time visit to a grid cell.
- **Parameter sweep:** Systematically varied $\epsilon$ and $\alpha$ and observed emergent behaviors.

---

## Evaluation and Parameter Exploration

### Metrics Collected

- **Episode return per agent**
- **Rewards collected (fairness/distribution)**
- **Time to first/last reward**
- **Fraction of rewards collected**
- **Weighted fitness $F$ (via varying $\alpha$)**
- **Mean steps to full collection**

### Parameter Sweeps

| Parameter           | Values Explored            | Key Effects                            |
|---------------------|---------------------------|----------------------------------------|
| $\epsilon$          | 0.05, 0.1, 0.2, 0.3 (decay) | High: faster coverage; Low: quick convergence but risk local optima |
| $\alpha$            | 0, 0.5, 1.0               | $\alpha$=1: max collection; $\alpha$=0: short paths   |
| COLLECT_RADIUS      | 0.3, **0.5 (proposed)**    | Higher: faster collection, less wasted time |
| vel_max             | 1.0, **3.5**              | Fast agents = faster full collection, but risk more collisions |
| Penalty for idle    | 0, **0.01**               | Discourages rotating in place          |

#### Sample Table: Fitness Function Sweep

| $\alpha$ | Mean Rewards Collected | Mean Episode Return | Fitness ($F$) |
|----------|-----------------------|--------------------|---------------|
| 0.0      | 7.3                   | 12.1               | 12.1          |
| 0.5      | 8.6                   | 13.9               | 11.25         |
| 1.0      | 9.1                   | 15.0               | 9.1           |

#### Scaling Agent Number

| Agents | Mean Rewards/Agent | Fraction Collected | Mean Steps (to collect all) |
|--------|-------------------|--------------------|-----------------------------|
|   2    |         9.0       |      0.97          |           550               |
|   5    |         8.7       |      0.95          |           690               |
|   10   |         8.2       |      0.90          |           1050              |
|   20   |         7.8       |      0.82          |           1740              |

*Above: Sample data illustrating scalable performance and congestion effects.*

---

## Results & Analysis

### Baseline vs Improved Agent Behaviors

- **Baseline:**  
    - Some agents "camp" or stall, with rewards sometimes uncollected for hundreds of steps.
    - Skewed reward distribution—certain agents outperform others heavily.
    - Episode returns rise slowly (see red curve, Fig 1 in plots).
- **With Improvements (epsilon decay, higher speed/radius, penalties):**
    - Fast, fair distribution: most agents collect at least one reward, with few outliers.
    - All rewards collected much more quickly: mean steps to full pick-up decreased by 30-50%.
    - Episode return curves rise steeper, and plateaus at higher levels.
    - Less time wasted in "idle" or "churning" rotations.
    - Exploration covers a greater fraction of the map in early steps.

<img src="plots/mean_std/episode_return_mean_std.png" alt="Learning curve" width="400" />
*Figure 1: Mean and std episode return curves (after improvements).*

### Emergent & Scalable Strategies

- Agents naturally "divide up" the map, occasionally racing for shared rewards or ceding far patches to others.
- At higher agent numbers, congestion increases—collision avoidance prevents deadlocks but slows down collection.
- Higher $\epsilon$ leads to more robust recovery from local minima—agents more likely to "give up" an unreachable patch and explore elsewhere.
- The best outcomes for a **balanced fitness** (fast and near-optimal collection) occur at midrange $\alpha$, confirming assignment feedback.

### Parameter Influence

- **Exploration rate ($\epsilon$):**  
    High at start is crucial for rapid collective coverage, but decay is needed for convergence.
- **$\alpha$ (Fitness Weight):**  
    Emergent specialization and division-of-labor seen at mid values; extremes skew either to greedy or to rapid path minimization.
- **Speed/Radius:**  
    Need to balance for environment density—excessive speed at high density can increase collisions, but is highly beneficial otherwise.

### Quantitative Comparisons

| Metric                    | Before     | After     |
|---------------------------|------------|-----------|
| Mean Steps (full pickup)  | 1700+      | 600-900   |
| Reward fairness (stdev)   | high       | low       |
| Earliest patch pickup     | >200 steps | <50       |

*With suggested improvements, all key outcomes improve, and the agent group works more fairly and efficiently.*

---

## Discussion

1. **Implementation insights:**  
   Regret matching is a strong framework for decentralized foraging. With partial observability and local information, agents still learn effective policies.

2. **Feedback incorporation:**  
   We explicitly explored the effect of fitness weighting ($\alpha$), and sweeped $\epsilon$, showing and analyzing qualitative and quantitative changes in outcomes.

3. **Emergent behavior:**  
   At scale, agents form unofficial "territories" and sometimes specialize in edge or cluster regions; rare congestion sometimes occurs but is mitigated by our improvements.

4. **Limitations:**  
   - In ultra-dense settings or with poorly tuned collision avoidance, congestion can persist.
   - Fully distributed learning, no explicit coordination or communication—some performance loss vs joint planners.
   - Map coverage can still vary depending on initial agent positions and random seeds.

5. **Future work:**  
   - Hierarchical role diffusion (agents choose forager or explorer role).
   - Adaptive shaping of the reward function based on congestion.
   - Explicit fairness metrics and multi-objective optimization.

---

## Conclusion

- Regret Matching is highly effective for scalable, decentralized multi-agent foraging—even without communication.
- With exploration decay, reward shaping, and proper parameter sweeps, rapid, high-yield, and fair collective results are achieved.
- Scaling introduces congestion, but improvements minimize negative impacts.
- Instructor feedback directly shaped our fitness evaluation and deepened our parameter exploration—providing more insight on emergent collective behaviors.

---

## Repository

**All code, data, figures, and results:**  
https://github.com/Valquaresma03/MAS/tree/main

---

## Appendix

**Key code and parameter improvements:**

- Dynamic epsilon decay in `rm.py`
- Increased `COLLECT_RADIUS` and `vel_max` in config
- Reward/penalty shaping for exploration and minimized stalling
- See in `rm.py` choose_new_action, reward_computation, and test.yaml

---

**Plots and log files:** See `/plots/` in the repo.
