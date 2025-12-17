import irsim


env = irsim.make("test.yaml")
env.load_behavior("rm")

for _ in range(5000):
    env.step()
    env.render(0.03)
    if env.done():
        break

env.end(3)










