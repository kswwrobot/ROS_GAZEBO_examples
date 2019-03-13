import torch
import torch.nn as nn
import torch.nn.functional as F


class Net(nn.Module):

    def __init__(self, input_dim, output_dim):
        super(Net, self).__init__()        
        self.fc1 = nn.Linear(input_dim, 5)
        self.fc2 = nn.Linear(5, 10)
        self.fc3 = nn.Linear(10, output_dim)

    def forward(self, x):
        # Max pooling over a (2, 2) window        
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.tanh(self.fc3(x)) * 50
        x = x.floor().int()
        return x

    def num_flat_features(self, x):
        size = x.size()[1:]  # all dimensions except the batch dimension
        num_features = 1
        for s in size:
            num_features *= s
        return num_features


