import irsim
from bully_FPSB import metrics_report

env = irsim.make("test_bully.yaml")
env.load_behavior("bully_FPSB")

for _ in range(500):
    env.step()
    env.reset_plot()
    env.render(0.03)
    if env.done():
        break

metrics_report()
env.end(3)










