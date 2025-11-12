import irsim
from ACO import metrics_report

env = irsim.make("test1.yaml")
env.load_behavior("ACO")

for _ in range(500):
    env.step()
    env.render(0.03)
    if env.done():
        break

metrics_report()
env.end(3)










