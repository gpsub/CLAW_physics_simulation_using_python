## Please install tensorflow, pyglet, pymunk library using pip before running this file
 
from env2 import ClawArms
from rl import DDPG

MAX_EPISODES = 200
MAX_EP_STEPS = 200
ON_TRAIN = True

# getting variables from env2.py
env = ClawArms()
s_dim = env.state_dim
a_dim = env.action_dim
a_bound = env.action_bound

# set RL method (continuous) sending the variables from env2.py to rl.py
rl = DDPG(a_dim, s_dim, a_bound)


def train():
    # start training
    for i in range(MAX_EPISODES):
        s = env.reset()# method in env2.py
        ep_r = 0.
        for j in range(MAX_EP_STEPS):
            env.render()  # used to update the window

            a = rl.choose_action(s) #getting prediction from rl.py

            s_, r, done = env.step(a) ## sending the prediction to env2.py and then check how much reward we got

            rl.store_transition(s, a, r, s_) ## storing the result in an array

            ep_r += r
            if rl.memory_full:
                # start to train once has fulfilled the memory
                rl.learn()

            s = s_
            if done or j == MAX_EP_STEPS-1:
                print('Ep: %i | %s | ep_r: %.1f | steps: %i' % (i, '---' if not done else 'done', ep_r, j))
                break
    rl.save()


def eval():
    rl.restore()
    env.render()
    env.viewer.set_vsync(True)
    while True:
        s = env.reset()
        for _ in range(200):
            env.render()
            a = rl.choose_action(s)
            s, r, done = env.step(a)
            if done:
                break


if ON_TRAIN:
    train()
else:
    eval()



