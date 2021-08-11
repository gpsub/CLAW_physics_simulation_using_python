import torch.nn as nn
import torch.nn.functional as F
## file for defining the neural network class
## consists of conv2d and fully connected layers to take the environmnet frames as input and output the 
# actions it wants to take,

class DQN(nn.Module):
    def __init__(self, in_channels=1, num_actions=18):
        ## in channels = number of channels, 
        ## num_actions  = output number of actions (12 actions or 4* 3 keys)
        ##
        super(DQN, self).__init__()
        self.conv1 = nn.Conv2d(in_channels, 32, kernel_size=8, stride=4)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=4, stride=2)
        self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1)
        self.fc4 = nn.Linear(7 * 7 * 64, 512)
        self.fc5 = nn.Linear(512, num_actions)

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.fc4(x.view(x.size(0), -1)))
        return self.fc5(x)