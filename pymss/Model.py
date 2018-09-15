import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T
import math
import time
from collections import OrderedDict
import numpy as np
from IPython import embed

MultiVariateNormal = torch.distributions.Normal
temp = MultiVariateNormal.log_prob
MultiVariateNormal.log_prob = lambda self, val: temp(self,val).sum(-1, keepdim=True)

temp2 = MultiVariateNormal.entropy
MultiVariateNormal.entropy = lambda self: temp2(self).sum(-1)
MultiVariateNormal.mode = lambda self: self.mean
class Container(nn.Module):
	def __init__(self,num):
		super(Container,self).__init__()
		self.num = num
		self.container = nn.Parameter(torch.zeros(self.num),requires_grad=False)
		self.counter = nn.Parameter(torch.zeros(1),requires_grad=False)

	def Push(self,val):

		self.container[int(self.counter.data[0].numpy().tolist())] = val
		self.counter += 1

	def Get(self):
		vec = self.container.detach().numpy()
		return vec[0:int(self.counter.data[0].numpy().tolist())]

class Model(nn.Module):
	def __init__(self,num_states,num_actions):
		super(Model,self).__init__()

		num_h1 = 64
		num_h2 = 32
		self.p_fc1 = nn.Linear(num_states,num_h1)
		self.p_fc2 = nn.Linear(num_h1,num_h2)
		self.p_fc3 = nn.Linear(num_h2,num_actions)
		self.log_std = nn.Parameter(torch.zeros(num_actions))

		self.v_fc1 = nn.Linear(num_states,num_h1)
		self.v_fc2 = nn.Linear(num_h1,num_h2)
		self.v_fc3 = nn.Linear(num_h2,1)
		self.reward_container = Container(10000)

		torch.nn.init.xavier_uniform_(self.p_fc1.weight)
		torch.nn.init.xavier_uniform_(self.p_fc2.weight)
		torch.nn.init.xavier_uniform_(self.p_fc3.weight)

		self.p_fc1.bias.data.zero_()
		self.p_fc2.bias.data.zero_()
		self.p_fc3.bias.data.zero_()

		torch.nn.init.xavier_uniform_(self.v_fc1.weight)
		torch.nn.init.xavier_uniform_(self.v_fc2.weight)
		torch.nn.init.xavier_uniform_(self.v_fc3.weight)

		self.v_fc1.bias.data.zero_()
		self.v_fc2.bias.data.zero_()
		self.v_fc3.bias.data.zero_()

		
	def forward(self,x):
		p_out = F.relu(self.p_fc1(x))
		p_out = F.relu(self.p_fc2(p_out))
		p_out = self.p_fc3(p_out)

		p_out = MultiVariateNormal(p_out,self.log_std.exp())

		v_out = F.relu(self.v_fc1(x))
		v_out = F.relu(self.v_fc2(v_out))
		v_out = self.v_fc3(v_out)
		return p_out,v_out

	def load(self,path):
		print('load nn {}'.format(path))
		self.load_state_dict(torch.load(path))

	def save(self,path):
		print('save nn {}'.format(path))
		torch.save(self.state_dict(),path)
		
	def get_action(self,s):
		ts = torch.tensor(s)
		p,_ = self.forward(ts)
		return p.loc.detach().numpy()

	def get_random_action(self,s):
		ts = torch.tensor(s)
		p,_ = self.forward(ts)
		return p.sample().detach().numpy()
# class Model(nn.Module):
# 	def __init__(self,num_states,num_actions):
# 		super(Model,self).__init__()

# 		hidden_layer_size1 = 64
# 		hidden_layer_size2 = 32

# 		self.basis = np.array([[0.0,-0.5,1.0,-0.5],[1.0,0.0,-2.5,1.5],[0.0,0.5,2.0,-1.5],[0.0,0.0,-0.5,0.5]],dtype=np.float32)

# 		self.p_fc0 = [nn.Linear(num_states-1,hidden_layer_size1),nn.Linear(num_states-1,hidden_layer_size1),nn.Linear(num_states-1,hidden_layer_size1),nn.Linear(num_states-1,hidden_layer_size1)]
# 		self.p_fc1 = [nn.Linear(hidden_layer_size1,hidden_layer_size2),nn.Linear(hidden_layer_size1,hidden_layer_size2),nn.Linear(hidden_layer_size1,hidden_layer_size2),nn.Linear(hidden_layer_size1,hidden_layer_size2)]
# 		self.p_fc2 = [nn.Linear(hidden_layer_size2,num_actions),nn.Linear(hidden_layer_size2,num_actions),nn.Linear(hidden_layer_size2,num_actions),nn.Linear(hidden_layer_size2,num_actions)]
	
# 		self.v_fc0 = [nn.Linear(num_states-1,hidden_layer_size1),nn.Linear(num_states-1,hidden_layer_size1),nn.Linear(num_states-1,hidden_layer_size1),nn.Linear(num_states-1,hidden_layer_size1)]
# 		self.v_fc1 = [nn.Linear(hidden_layer_size1,hidden_layer_size2),nn.Linear(hidden_layer_size1,hidden_layer_size2),nn.Linear(hidden_layer_size1,hidden_layer_size2),nn.Linear(hidden_layer_size1,hidden_layer_size2)]
# 		self.v_fc2 = [nn.Linear(hidden_layer_size2,1),nn.Linear(hidden_layer_size2,1),nn.Linear(hidden_layer_size2,1),nn.Linear(hidden_layer_size2,1)]

# 		self.register_parameter('p_w0_0',self.p_fc0[0].weight)
# 		self.register_parameter('p_w1_0',self.p_fc1[0].weight)
# 		self.register_parameter('p_w2_0',self.p_fc2[0].weight)
# 		self.register_parameter('p_w0_1',self.p_fc0[1].weight)
# 		self.register_parameter('p_w1_1',self.p_fc1[1].weight)
# 		self.register_parameter('p_w2_1',self.p_fc2[1].weight)
# 		self.register_parameter('p_w0_2',self.p_fc0[2].weight)
# 		self.register_parameter('p_w1_2',self.p_fc1[2].weight)
# 		self.register_parameter('p_w2_2',self.p_fc2[2].weight)
# 		self.register_parameter('p_w0_3',self.p_fc0[3].weight)
# 		self.register_parameter('p_w1_3',self.p_fc1[3].weight)
# 		self.register_parameter('p_w2_3',self.p_fc2[3].weight)

# 		self.register_parameter('v_w0_0',self.v_fc0[0].weight)
# 		self.register_parameter('v_w1_0',self.v_fc1[0].weight)
# 		self.register_parameter('v_w2_0',self.v_fc2[0].weight)
# 		self.register_parameter('v_w0_1',self.v_fc0[1].weight)
# 		self.register_parameter('v_w1_1',self.v_fc1[1].weight)
# 		self.register_parameter('v_w2_1',self.v_fc2[1].weight)
# 		self.register_parameter('v_w0_2',self.v_fc0[2].weight)
# 		self.register_parameter('v_w1_2',self.v_fc1[2].weight)
# 		self.register_parameter('v_w2_2',self.v_fc2[2].weight)
# 		self.register_parameter('v_w0_3',self.v_fc0[3].weight)
# 		self.register_parameter('v_w1_3',self.v_fc1[3].weight)
# 		self.register_parameter('v_w2_3',self.v_fc2[3].weight)

# 		self.register_parameter('p_b0_0',self.p_fc0[0].bias)
# 		self.register_parameter('p_b0_1',self.p_fc0[1].bias)
# 		self.register_parameter('p_b0_2',self.p_fc0[2].bias)
# 		self.register_parameter('p_b0_3',self.p_fc0[3].bias)
# 		self.register_parameter('p_b1_0',self.p_fc1[0].bias)
# 		self.register_parameter('p_b1_1',self.p_fc1[1].bias)
# 		self.register_parameter('p_b1_2',self.p_fc1[2].bias)
# 		self.register_parameter('p_b1_3',self.p_fc1[3].bias)
# 		self.register_parameter('p_b2_0',self.p_fc2[0].bias)
# 		self.register_parameter('p_b2_1',self.p_fc2[1].bias)
# 		self.register_parameter('p_b2_2',self.p_fc2[2].bias)
# 		self.register_parameter('p_b2_3',self.p_fc2[3].bias)

# 		self.register_parameter('v_b0_0',self.v_fc0[0].bias)
# 		self.register_parameter('v_b0_1',self.v_fc0[1].bias)
# 		self.register_parameter('v_b0_2',self.v_fc0[2].bias)
# 		self.register_parameter('v_b0_3',self.v_fc0[3].bias)
# 		self.register_parameter('v_b1_0',self.v_fc1[0].bias)
# 		self.register_parameter('v_b1_1',self.v_fc1[1].bias)
# 		self.register_parameter('v_b1_2',self.v_fc1[2].bias)
# 		self.register_parameter('v_b1_3',self.v_fc1[3].bias)
# 		self.register_parameter('v_b2_0',self.v_fc2[0].bias)
# 		self.register_parameter('v_b2_1',self.v_fc2[1].bias)
# 		self.register_parameter('v_b2_2',self.v_fc2[2].bias)
# 		self.register_parameter('v_b2_3',self.v_fc2[3].bias)

# 		torch.nn.init.xavier_uniform_(self.state_dict()['p_w0_0'])
# 		torch.nn.init.xavier_uniform_(self.state_dict()['p_w1_0'])
# 		torch.nn.init.xavier_uniform_(self.state_dict()['p_w2_0'])
# 		self.state_dict()['p_b0_0'].zero_()
# 		self.state_dict()['p_b1_0'].zero_()
# 		self.state_dict()['p_b2_0'].zero_()

# 		self.p_fc0[1].load_state_dict(self.p_fc0[0].state_dict())
# 		self.p_fc0[2].load_state_dict(self.p_fc0[0].state_dict())
# 		self.p_fc0[3].load_state_dict(self.p_fc0[0].state_dict())
# 		self.p_fc1[1].load_state_dict(self.p_fc1[0].state_dict())
# 		self.p_fc1[2].load_state_dict(self.p_fc1[0].state_dict())
# 		self.p_fc1[3].load_state_dict(self.p_fc1[0].state_dict())
# 		self.p_fc2[1].load_state_dict(self.p_fc2[0].state_dict())
# 		self.p_fc2[2].load_state_dict(self.p_fc2[0].state_dict())
# 		self.p_fc2[3].load_state_dict(self.p_fc2[0].state_dict())

		
# 		torch.nn.init.xavier_uniform_(self.state_dict()['v_w0_0'])
# 		torch.nn.init.xavier_uniform_(self.state_dict()['v_w1_0'])
# 		torch.nn.init.xavier_uniform_(self.state_dict()['v_w2_0'])
# 		self.state_dict()['v_b0_0'].zero_()
# 		self.state_dict()['v_b1_0'].zero_()
# 		self.state_dict()['v_b2_0'].zero_()

# 		self.v_fc0[1].load_state_dict(self.v_fc0[0].state_dict())
# 		self.v_fc0[2].load_state_dict(self.v_fc0[0].state_dict())
# 		self.v_fc0[3].load_state_dict(self.v_fc0[0].state_dict())
# 		self.v_fc1[1].load_state_dict(self.v_fc1[0].state_dict())
# 		self.v_fc1[2].load_state_dict(self.v_fc1[0].state_dict())
# 		self.v_fc1[3].load_state_dict(self.v_fc1[0].state_dict())
# 		self.v_fc2[1].load_state_dict(self.v_fc2[0].state_dict())
# 		self.v_fc2[2].load_state_dict(self.v_fc2[0].state_dict())
# 		self.v_fc2[3].load_state_dict(self.v_fc2[0].state_dict())

# 		self.log_std = nn.Parameter(torch.zeros(num_actions))
			
# 		self.reward_container = Container(10000)

# 	def forward(self,x):
# 		s,phi = torch.split(x,[x.shape[1]-1,1],dim=1)
# 		phi = phi.detach().numpy().reshape(-1)
# 		nslice = (4.0*phi)

# 		i1 = np.floor(nslice).astype(np.int32)%4

# 		ones = np.ones(phi.shape[0])
# 		alpha = nslice%1.0
# 		alpha2 = alpha*alpha
# 		alpha3 = alpha*alpha*alpha
# 		a = np.vstack([ones,alpha,alpha2,alpha3])
# 		a = (self.basis@a).transpose()

# 		for i in range(len(a)):
# 			a[i] = np.roll(a[i],i1[i]+3)
# 		a = torch.tensor(a.transpose().astype(np.float32))

# 		p_out = torch.mul(self.p_fc0[0](s),a[0][:,None]) + torch.mul(self.p_fc0[1](s),a[1][:,None]) + torch.mul(self.p_fc0[2](s),a[2][:,None]) + torch.mul(self.p_fc0[3](s),a[3][:,None])
# 		p_out = F.relu(p_out)
# 		p_out = torch.mul(self.p_fc1[0](p_out),a[0][:,None]) + torch.mul(self.p_fc1[1](p_out),a[1][:,None]) + torch.mul(self.p_fc1[2](p_out),a[2][:,None]) + torch.mul(self.p_fc1[3](p_out),a[3][:,None])
# 		p_out = F.relu(p_out)
# 		p_out = torch.mul(self.p_fc2[0](p_out),a[0][:,None]) + torch.mul(self.p_fc2[1](p_out),a[1][:,None]) + torch.mul(self.p_fc2[2](p_out),a[2][:,None]) + torch.mul(self.p_fc2[3](p_out),a[3][:,None])
# 		p_out = MultiVariateNormal(p_out,self.log_std.exp())

# 		v_out = torch.mul(self.v_fc0[0](s),a[0][:,None]) + torch.mul(self.v_fc0[1](s),a[1][:,None]) + torch.mul(self.v_fc0[2](s),a[2][:,None]) + torch.mul(self.v_fc0[3](s),a[3][:,None])
# 		v_out = F.relu(v_out)
# 		v_out = torch.mul(self.v_fc1[0](v_out),a[0][:,None]) + torch.mul(self.v_fc1[1](v_out),a[1][:,None]) + torch.mul(self.v_fc1[2](v_out),a[2][:,None]) + torch.mul(self.v_fc1[3](v_out),a[3][:,None])
# 		v_out = F.relu(v_out)
# 		v_out = torch.mul(self.v_fc2[0](v_out),a[0][:,None]) + torch.mul(self.v_fc2[1](v_out),a[1][:,None]) + torch.mul(self.v_fc2[2](v_out),a[2][:,None]) + torch.mul(self.v_fc2[3](v_out),a[3][:,None])

# 		return p_out,v_out
# 	def load(self,path):
# 		print('load nn {}'.format(path))
# 		print(path)
# 		self.load_state_dict(torch.load(path))
# 	def save(self,path):
# 		print('save nn {}'.format(path))
# 		torch.save(self.state_dict(),path)
		
# 	def get_action(self,x):
# 		p,_ = self.forward(torch.tensor(x.reshape(1,-1)))
# 		return p.loc.detach().numpy()

# 	def get_random_action(self,x):
# 		p,_ = self.forward(torch.tensor(x.reshape(1,-1)))
# 		return p.sample().detach().numpy()

class MuscleNN(nn.Module):
	def __init__(self,num_input,num_output):
		super(MuscleNN,self).__init__()

		self.num_input = num_input
		self.num_output = num_output
		self.num_h1 = 128
		self.num_h2 = 128

		self.fc1 = nn.Linear(self.num_input,self.num_h1)
		self.fc2 = nn.Linear(self.num_h1,self.num_h2)
		self.fc3 = nn.Linear(self.num_h2,self.num_output)
		self.training_loss_container = Container(50000)
		self.evaluation_loss_container = Container(50000)
		
		self.mean_input = Container(self.num_input)
		self.std_input = Container(self.num_input)
		# self.mean_output = Container(self.num_output)
		# self.std_output = Container(self.num_output)

		torch.nn.init.xavier_uniform_(self.fc1.weight)
		torch.nn.init.xavier_uniform_(self.fc2.weight)
		torch.nn.init.xavier_uniform_(self.fc3.weight)

		self.fc1.bias.data.zero_()
		self.fc2.bias.data.zero_()
		self.fc3.bias.data.zero_()

	def SetNormalizer(self,mi,si):		
		for x in mi:
			self.mean_input.Push(x.tolist())
		for x in si:
			self.std_input.Push(x.tolist())
		# for x in mo:
			# self.mean_output.Push(x.tolist())
		# for x in so:
			# self.std_output.Push(x.tolist())

	def forward(self,x):
		x = (x - self.mean_input.container)/self.std_input.container

		out = F.relu(self.fc1(x))
		out = F.relu(self.fc2(out))
		out = F.tanh(self.fc3(out))

		# out = (out - self.mean_output.container)/self.std_output.container
		out = 0.5*(out+1)
		return out
	def get_activation(self,x):
		act = self.forward(torch.tensor(x.reshape(1,-1)))
		return act.detach().numpy()

	def load(self,path):
		print('load nn {}'.format(path))
		self.load_state_dict(torch.load(path))
		
	def save(self,path):
		print('save nn {}'.format(path))
		torch.save(self.state_dict(),path)