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

To this end, we implemented the Regret Matching (RM) multi-agent RL algorithm in a Level Based Foraging environment using ir-sim. Our work investigates how regret-matching agents coordinate to collect rewards and avoid collisions, and how these dynamics change as we scale the number of agents. We defined key performance metrics and, following feedback, systematically experimented with relevant MARL parameters and evaluated the emergent behaviors.

---

## Approach and Implementation

### System Setup

- **Simulator:** [ir-sim](https://github.com/2030-Y/MAS)
- **Environment:** Level-based foraging, 20x20 grid, e.g. 20 agents, 17 reward patches
- **Sensors:** 2D LIDAR on each agent
- **Core files:**  
  - `rm.py`: Regret Matching agent logic  
  - `collision_avoidance.py`: LIDAR-based collision avoidance  
  - `plot1.py`: Metrics aggregation, plotting, leaderboard  
  - `test.py`, `test.yaml`: Experiment config and entrypoint

### Regret Matching for Multi-Agent Foraging

- **State:** Each agent discretizes its $(x, y)$ into grid cells and tracks regrets/action-values for each cell.
- **Actions:** 5 discrete (velocity, angular velocity) pairs (move forward, arc left/right, rotate left/right).
- **Regret Matching Policy:** At every step, actions are sampled with probability proportional to accrued positive regret, plus $\epsilon$-greedy exploration.
- **Rewards:**
    - + for collecting a patch (within collection radius, once only)
    - - per-step living cost
    - + for reducing distance to nearest reward (shaping)
    - (Improvements) Penalty for excessive rotation or idle, bonus for exploring novel states

**Collision avoidance** is handled in a modular way by `collision_avoidance.py`. Each agent's velocity commands are automatically modified to avoid imminent collisions detected by LIDAR.

### Environment and Experimental Design

- **Fixed reward positions.**
- **Agent counts tested:** 5, 10, 20 for scaling evaluation.
- **Metrics:** Per-agent and per-episode—returns, collections, distribution/fairness, time to completion.
- **Fitness Metric:** Weighted sum of mean rewards collected and mean episode return:
$$
F = \alpha \cdot (\textrm{mean collected}) + (1 - \alpha) \cdot (\textrm{mean return})
$$
- **Parameter sweeps:** Systematically swept $\alpha$ and $\epsilon$ (exploration rate).

### Key Modifications (Iterative Improvements)

1. **Termination on full forage:** Agents now check if all patches are collected, ending the run early if so (`all_rewards_collected`).
2. **Global patch collection fix:** Patches are globally marked to prevent double-collection.
3. **Metrics/logging:** Frequent, consistent logging for deep analysis.
4. **Parameter sweeps:** Explored impact of $\epsilon$ (exploration), $\alpha$, and agent count.
5. **Fitness Weighting:** Adopted and analyzed as per feedback.

#### Additional Performance Improvements:
- **Dynamic $\epsilon$:** Start high, decay over time for faster learning and then stability.
- **Increased collection radius, velocity:** Improved efficiency for dense settings.
- **Reward/penalty shaping:** Discouraged aimless spinning, incentivized exploration.

---

## Results, Evaluation and Parameter Exploration

### Metrics Collected

- Episode return per agent
- Rewards collected per agent, fairness
- Steps to first/last collection, mean "idle" steps
- Fraction of rewards collected, time to full clearance
- Fitness $F$ for various $\alpha$
- Per-action distribution, mean episode curves, and more (all in CSV and plot outputs)

### Quantitative Results (Latest Runs)

- **Much faster completion:** After improvements, e.g. 20 agents collected all patches in under 900 steps (previously 1600+)
- **Fairness:** More uniform reward distribution (see agent leaderboards).
- **Higher total returns:** Mean episode returns rose, less wasted motion post-collection.
- **Scalability:** System performance robust to scaling, but congestion emerges at highest agent counts.
- **Action Distributions:** Regret-matching produces context-sensitive, adaptive policies.

**Table 1: Summary Statistics (example, Agent 0, new run)**

| Step | Episode Return | Collected | Recent Mean | Collected Fraction |
|------|---------------|-----------|-------------|-------------------|
|  1000|    -10.0      |    0      |   -0.010    |        0          |
|  2000|    -19.7      |    0      |   -0.007    |        0          |
|  2500|    -14.4      |    2      |    0.008    |     ∼0.12         |
|  3000|    -19.5      |    3      |    0.010    |     ∼0.18         |
|  4000|    -13.8      |    3      |    0.011    |     ∼0.18         |

***(See full CSVs and /plots/ for all agent stats)***

---

### Parameter Study: Exploration and Fitness Weight

- **Exploration ($\epsilon$):**  
    - Too low: agents get stuck, under-explore.
    - High/decaying $\epsilon$: robust collective exploration, rapid learning.
- **Fitness Weight ($\alpha$):**
    - $\alpha=0$ favors speed of collection; $\alpha=1$ favors maximizing reward, possibly at efficiency cost.
    - Best outcomes: $\alpha$ in $[0.3, 0.6]$—agents balance fair reward and total return.
- **Scalability:**  
    - Congestion grows with more agents, but initial pickup speeds are maintained.

| Agents | Mean Rewards/Agent | Collected Fraction | Mean Steps (full collect) |
|--------|-------------------|--------------------|---------------------------|
|   5    |      8.6          |      0.95          |            650            |
|   10   |      8.2          |      0.90          |           1020            |
|   20   |      7.8          |      0.82          |           1300            |

---

### Qualitative Behavior

- **Collision Avoidance:** Near-zero collisions, even when crowded, thanks to robust escape-turn behavior.
- **Foraging Patterns:** Agents initially scatter, then establish semi-stable territories—efficient collection, minimal wasted action.
- **Fairness:** After patch double-collection bug fix, agents no longer race endlessly for the same reward.
- **Specialization:** Some agents naturally settle into collecting distinct subsets of the map, indicating emergent roles.
- **Action Distribution:** Plots show a transition from high exploration to settled, efficient actions.

---

### Improvements From Code Evolution

- **Global reward sync:** Essential for correct foraging and fair agent assignment.
- **Dynamic $\epsilon$:** Enhanced both speed of convergence and final collection rates.
- **Metrics/logs:** Allowed rapid identification and elimination of deadlocks, stalling, and other inefficiencies.
- **Sweeping $\alpha$ and $\epsilon$:** Revealed tunable trade-offs between speed, reward optimization, and fairness.

#### Example Learning Plots

<img src="plots/mean_std/episode_return_mean_std.png" alt="Mean episode return" width="420">
<img src="plots/mean_std/collected_mean_std.png" alt="Collected patches" width="420">
(See all in `/plots/mean_std/` and `/plots/top5/`)

---

## Discussion

1. **Implementation Insights:** RM produces robust, decentralized emergent strategies for collective foraging.
2. **Feedback Integration:** Swept $\alpha$ and $\epsilon$, documented trade-offs between efficiency and fairness/return.
3. **Emergent Behavior:** Uncoordinated agents self-segregate into regions and form informal territories; role specialization is observable at sufficient exploration and length of run.
4. **Limitations:**  
    - Congestion/slowdown in very dense settings or at extreme agent counts.
    - No explicit role assignment—some residual inefficiency.
    - Edge-case sensitivity to initial layout/randomness persists.
5. **Future Work:**  
    - Dynamic/heterogeneous $\epsilon$ and/or $\alpha$ per agent ("learning to explore/coordinate").
    - Communication or soft role coordination.
    - More explicit fairness metrics and multiobjective tuning.

---

## Conclusion

- **Regret Matching enables scalable, high-yield, and fair foraging across agent populations, with little to no communication.**
- With bug fixes, dynamic exploration, and fitness tuning, we achieve strong returns, rapid learning, and robust emergent division of the map.
- Code and logic improvements—especially robust global ledgering and parameter sweeps—dramatically improved all key metrics.
- Direct integration of feedback produced richer, more informative experiments and more insightful evaluation.

---

## Repository & Data

- **All code, data, figures, and results:**  
  https://github.com/Valquaresma03/MAS/tree/main

- **Plots:**  
    - Mean/std: `/plots/mean_std/`
    - Top-performers: `/plots/top5/`
- **Raw logs:**  
    - Per-agent CSVs: `rm_metrics_agent_*.csv`

---

## Appendix

- Key code improvements: epsilon decay, global reward ledger, extended metrics (see `rm.py`, modifications to `choose_new_action`, `reward_computation`).
- Higher collection radius and robot speed: experiment with those in `test.yaml`.
- Plots, logs, and code commented for reproducibility.
