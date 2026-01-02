import irsim
import rm
from rm import set_reward_patches, all_rewards_collected
import os

RESULTS_DIR = "results_20agents"
os.makedirs(RESULTS_DIR, exist_ok=True)  # Create if needed
os.chdir(RESULTS_DIR)  # Change working directory to results folder
env = irsim.make("test.yaml")
env.load_behavior("rm")

for step in range(5000):
    env.step()
    env.render(0.03)
    # Print how many rewards were collected so far
    print(f"STEP {step} - Collected: {len(rm.GLOBAL_COLLECTED_REWARDS)} / {len(rm.REWARD_PATCHES)}")
    if env.done():
        print("Environment returned done=True, stopping.")
        break
    if rm.all_rewards_collected():
        print("All reward patches collected, ending early.")
        break

env.end(3)
if not rm.all_rewards_collected():
    print(f"Simulation ended without collecting all rewards: {len(rm.GLOBAL_COLLECTED_REWARDS)} / {len(rm.REWARD_PATCHES)}")
else:
    print("All rewards collected!")