import math
import random
import time
import os
import sys

import collections
from collections import namedtuple
from collections import deque
from itertools import count

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T

import numpy as np
from pymss import Env
from IPython import embed
from Model import *

Episode = namedtuple('Episode',('s','a','r', 'value', 'logprob'))
class EpisodeBuffer(object):
	def __init__(self):
		self.data = []

	def Push(self, *args):
		self.data.append(Episode(*args))

	def GetData(self):
		return self.data

Transition = namedtuple('Transition',('s','a', 'logprob', 'TD', 'GAE'))
class ReplayBuffer(object):
	def __init__(self, buff_size = 10000):
		super(ReplayBuffer, self).__init__()
		self.buffer = deque(maxlen=buff_size)

	def Sample(self, batch_size):
		index_buffer = [i for i in range(len(self.buffer))]
		indices = random.sample(index_buffer, batch_size)
		return indices, self.np_buffer[indices] 

	def Push(self,*args):
		self.buffer.append(Transition(*args))

	def Clear(self):
		self.buffer.clear()

class PPO(object):
	def __init__(self,num_slaves):
		np.random.seed(seed = int(time.time()))
		self.env = Env(num_slaves)
		self.num_slaves = num_slaves
		self.num_state = self.env.GetNumState()
		self.num_action = self.env.GetNumAction()
		self.num_epochs = 10
		self.num_evaluation = 0
		self.num_tuple_so_far = 0
		self.num_episode = 0
		self.num_tuple = 0

		self.gamma = 0.99
		self.lb = 0.95
		self.clip_ratio = 0.2
		
		self.buffer_size = 4096
		self.batch_size = 256
		self.replay_buffer = ReplayBuffer(40000)

		self.model = Model(self.num_state,self.num_action)
		self.optimizer = optim.Adam(self.model.parameters(),lr=7E-4)

		self.w_entropy = 0.0

		self.loss_actor = 0.0
		self.loss_critic = 0.0
		self.sum_return = 0.0
		
		self.tic = time.time()

	def SaveModel(self):
		self.model.save('../nn/'+str(self.num_evaluation)+'.pt')

	def LoadModel(self,model_path):
		self.model.load(model_path)
		self.num_evaluation = int(self.model.reward_container.counter.data[0].numpy().tolist())

	def ComputeTDandGAE(self):
		self.replay_buffer.Clear()
		self.sum_return = 0.0
		for epi in self.total_episodes:
			data = epi.GetData()
			size = len(data)
			states, actions, rewards, values, logprobs = zip(*data)

			values = np.concatenate((values, np.zeros(1)), axis=0)
			advantages = np.zeros(size)
			ad_t = 0

			epi_return = 0.0
			for i in reversed(range(len(data))):
				epi_return += rewards[i]
				delta = rewards[i] + values[i+1] * self.gamma - values[i]
				ad_t = delta + self.gamma * self.lb * ad_t
				advantages[i] = ad_t
			self.sum_return += epi_return
			TD = values[:size] + advantages
			
			for i in range(size):
				self.replay_buffer.Push(states[i], actions[i], logprobs[i], TD[i], advantages[i])
		self.num_episode = len(self.total_episodes)
		self.num_tuple = len(self.replay_buffer.buffer)
		
		self.num_tuple_so_far += self.num_tuple

	def GenerateTransitions(self):
		self.total_episodes = []
		states = [None]*self.num_slaves
		actions = [None]*self.num_slaves
		rewards = [None]*self.num_slaves
		states_next = [None]*self.num_slaves
		episodes = [None]*self.num_slaves
		for j in range(self.num_slaves):
			episodes[j] = EpisodeBuffer()
		self.env.Resets(True)
		states = self.env.GetStates()
		local_step = 0
		terminated = [False]*self.num_slaves
		while True:
			a_dist,v = self.model(torch.tensor(states))
			actions = a_dist.sample().detach().numpy()
			logprobs = a_dist.log_prob(torch.tensor(actions)).detach().numpy().reshape(-1)
			values = v.detach().numpy().reshape(-1)

			self.env.SetActions(actions)
			self.env.Steps()
			for j in range(self.num_slaves):
				if terminated[j]:
					continue

				nan_occur = False
				if np.any(np.isnan(states[j])) or np.any(np.isnan(actions[j])) or np.any(np.isnan(values[j])) or np.any(np.isnan(logprobs[j])):
					nan_occur = True
				else:
					rewards[j] = self.env.GetReward(j)
					episodes[j].Push(states[j], actions[j], rewards[j], values[j], logprobs[j])
					local_step += 1
				# if episode is terminated
				if self.env.IsTerminalState(j) or (nan_occur is True):
					# push episodes
					self.total_episodes.append(episodes[j])

					# if data limit is exceeded, stop simulations
					if local_step < self.buffer_size:
						episodes[j] = EpisodeBuffer()
						self.env.Reset(True,j)
					else:
						terminated[j] = True
			if local_step >= self.buffer_size:
				all_terminated = True
				for j in range(self.num_slaves):
					if terminated[j] is False:
						all_terminated = False

				if all_terminated is True:
					break
			
			states = self.env.GetStates()
		
	def OptimizeModel(self):

		self.ComputeTDandGAE()
		all_transitions = np.array(self.replay_buffer.buffer)
		for _ in range(self.num_epochs):
			np.random.shuffle(all_transitions)
			for i in range(len(all_transitions)//self.batch_size):
				transitions = all_transitions[i*self.batch_size:(i+1)*self.batch_size]
				batch = Transition(*zip(*transitions))

				stack_s = np.vstack(batch.s).astype(np.float32)
				stack_a = np.vstack(batch.a).astype(np.float32)
				stack_lp = np.vstack(batch.logprob).astype(np.float32)
				stack_td = np.vstack(batch.TD).astype(np.float32)
				stack_gae = np.vstack(batch.GAE).astype(np.float32)
				
				a_dist,v = self.model(torch.tensor(stack_s))
				'''Critic Loss'''
				loss_critic = ((v-torch.tensor(stack_td)).pow(2)).mean()
				
				'''Actor Loss'''
				ratio = torch.exp(a_dist.log_prob(torch.tensor(stack_a))-torch.tensor(stack_lp))
				stack_gae = (stack_gae-stack_gae.mean())/(stack_gae.std()+ 1E-5)
				surrogate1 = ratio * torch.tensor(stack_gae)
				surrogate2 = torch.clamp(ratio,min =1.0-self.clip_ratio,max=1.0+self.clip_ratio) * torch.tensor(stack_gae)
				loss_actor = - torch.min(surrogate1,surrogate2).mean()
				'''Entropy Loss'''
				loss_entropy = - self.w_entropy * a_dist.entropy().mean()

				self.loss_actor = loss_actor.detach().numpy().tolist()
				self.loss_critic = loss_critic.detach().numpy().tolist()
				
				loss = loss_actor + loss_entropy + loss_critic

				self.optimizer.zero_grad()
				loss.backward(retain_graph=True)
				for param in self.model.parameters():
					if param.grad is not None:
						param.grad.data.clamp_(-0.5,0.5)
				self.optimizer.step()

	def Train(self):		
		self.GenerateTransitions()
		self.OptimizeModel()

	def Evaluate(self):
		print('# {} (time : {:.2f}s)'.format(self.num_evaluation,time.time() - self.tic))
		print('||Noise                    : {:.3f}'.format(self.model.log_std.exp().mean()))
		print('||Loss Actor               : {:.4f}'.format(self.loss_actor))
		print('||Loss Critic              : {:.4f}'.format(self.loss_critic))
		print('||Num Transition So far    : {}'.format(self.num_tuple_so_far))
		print('||Num Transition           : {}'.format(self.num_tuple))
		print('||Num Episode              : {}'.format(self.num_episode))
		print('||Avg Return per episode   : {:.3f}'.format(self.sum_return/self.num_episode))
		print('||Avg Reward per transition: {:.3f}'.format(self.sum_return/self.num_tuple))
		self.model.reward_container.Push(self.sum_return/self.num_episode)
		self.SaveModel()
		self.num_evaluation = int(self.model.reward_container.counter.data[0].numpy().tolist())
		print('=============================================')

		return self.model.reward_container.Get()
		
import matplotlib
import matplotlib.pyplot as plt
plt.ion()

def Plot(y,title,num_fig=1,ylim=True):
	temp_y = np.zeros(y.shape)
	if y.shape[0]>5:
		temp_y[0] = y[0]
		temp_y[1] = 0.5*(y[0] + y[1])
		temp_y[2] = 0.3333*(y[0] + y[1] + y[2])
		temp_y[3] = 0.25*(y[0] + y[1] + y[2] + y[3])
		for i in range(4,y.shape[0]):
			temp_y[i] = np.sum(y[i-4:i+1])*0.2

	plt.figure(num_fig)
	plt.clf()
	plt.hold(True)
	plt.title(title)
	plt.plot(y,'b')
	
	plt.plot(temp_y,'r')

	plt.show()
	if ylim:
		plt.ylim([0,1])
	plt.pause(0.001)


import argparse

if __name__=="__main__":
	ppo = PPO(16)

	parser = argparse.ArgumentParser()
	parser.add_argument('-m','--model',help='actor model directory')
	args =parser.parse_args()
	if args.model is not None:
		ppo.LoadModel(args.model)
	rewards = []
	print('num states: {}, num actions: {}'.format(ppo.env.GetNumState(),ppo.env.GetNumAction()))
	for i in range(50000):
		ppo.Train()
		Plot(ppo.Evaluate(),'reward',1,False)