from mod_code.phy_env_main import Simulator





sim = Simulator()
sim.main()
reward  = sim.ret_reward()
print(reward)



##***************Testing environment architecture************
## TODO:: Stuff to be updated so that we can make the RL model work now that the physics environment is ready
##**********phy_env file
## TODO: need a render method(phy_env.py)env.render() DONE
## TODO: already have env.reset(phy_env.py) DONE
## TODO: need env.step([action]) function(phy_env.py), env.step returns new state,reward and done



### RL files:
## TODO: need choose_action(state),returns chosen action
## TODO: need learn(), learns the neuralnetwork weights
## TODO: will make a class of DQN which has the functions for replay buffer, etc etc
## TODO: need a store transition(state,reward,action taken, next state) function 
## TODO: if memory full then learn

## will call both rl files and activate simulation here itself. Will test out different models and see which works
# best.
#**************************************************

#******************Real world architecture**********************
# will be dealt with later, will use aws to connect with robot and send predictions

## real thing is in real life.. have to make keeping that in mind
## so now this is the main file, will start the simulator and will be the linking between the RL model and the simulator
## in the real world:
## 1. get the image, state from AWS
## 2. get the action prediction from the RL model,
## 3. pass that prediction to the model/motor and see it move,get the reward and then same process



