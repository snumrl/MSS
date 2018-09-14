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
from pymss import Preprocess
from IPython import embed
from Model import *


class Regressor(object):
	def __init__(self,num_slaves):
		np.random.seed(seed = int(time.time()))
		self.env = Preprocess(num_slaves)
		self.num_slaves = num_slaves
		self.num_input = self.env.GetNumInput()
		self.num_output = self.env.GetNumOutput()
		
		self.num_epoch = 1
		self.num_minibatch = 64
		self.num_evaluation = 0
		self.num_pair_per_simulation = 2

		self.model = MuscleNN(self.num_input,self.num_output)
		normalizer = self.env.GetNormalizer()
		self.model.SetNormalizer(normalizer[0][0:self.num_input],normalizer[1][0:self.num_input],normalizer[0][-self.num_output:],normalizer[1][-self.num_output:])
		
		# self.model.SetNormalizer()
		self.optimizer = optim.Adam(self.model.parameters(),lr=1E-5)

		self.loss = 0.0
		self.buffer = np.zeros(shape=(0,self.num_input+self.num_output),dtype=np.float32)
		self.tic = time.time()

	def SaveModel(self):
		self.model.save('../nn_muscle/'+str(self.num_evaluation)+'.pt')

	def LoadModel(self,model_path):
		self.model.load(model_path)
		self.num_evaluation = int(self.model.evaluation_loss_container.counter.data[0].numpy().tolist())

	def GeneratePairs(self,num):
		count = num//self.num_slaves
		count = count//self.num_pair_per_simulation
		self.buffer = np.zeros(shape=(0,self.num_input+self.num_output),dtype=np.float32)
		for i in range(count):
			self.env.GeneratePairs(self.num_pair_per_simulation)
			self.buffer = np.vstack([self.buffer,self.env.Get()])
	def Evaluate(self,num):
		temp_buffer = np.zeros(shape=(0,self.num_input+self.num_output),dtype=np.float32)
		for i in range(num):
			self.env.GeneratePairs(self.num_pair_per_simulation)
			temp_buffer = np.vstack([temp_buffer,self.env.Get()])
		X,Y,_ = np.split(temp_buffer,[self.num_input,self.num_input+self.num_output],axis=1)
		G_x = self.model(torch.tensor(X))
		loss = ((G_x-torch.tensor(Y)).pow(2)).mean()
		self.num_evaluation += 1
		self.model.evaluation_loss_container.Push(loss.data.tolist())
		self.SaveModel()
	def OptimizeModel(self):
		for _ in range(self.num_epoch):
			np.random.shuffle(self.buffer)
			for i in range(len(self.buffer)//self.num_minibatch):
				pairs = self.buffer[i*self.num_minibatch:(i+1)*self.num_minibatch]
				X,Y,_ = np.split(pairs,[self.num_input,self.num_input+self.num_output],axis=1)
				G_x = self.model(torch.tensor(X))
				loss = ((G_x-torch.tensor(Y)).pow(2)).mean()
				self.loss = loss.data.tolist()
				
					
				self.optimizer.zero_grad()
				loss.backward(retain_graph=True)
				for param in self.model.parameters():
					if param.grad is not None:
						param.grad.data.clamp_(-0.5,0.5)
				self.optimizer.step()
		self.model.training_loss_container.Push(self.loss)
		self.Evaluate(1)


import matplotlib
import matplotlib.pyplot as plt
plt.ion()

def Plot(y,y1,title,num_fig=1,ylim=True):
	plt.figure(num_fig)
	plt.clf()
	
	plt.title(title)
	plt.hold(True)	
	if(len(y)>100):
		plt.plot(y[len(y)-100:],'b')
		plt.plot(y1[len(y1)-100:],'r')
	else:
		plt.plot(y,'b')
		plt.plot(y1,'r')
	plt.show()
	if ylim:
		plt.ylim([0,12])
	plt.pause(0.001)
import argparse

if __name__=="__main__":
	rg = Regressor(16)
	parser = argparse.ArgumentParser()
	parser.add_argument('-m','--model',help='actor model directory')
	args =parser.parse_args()
	if args.model is not None:
		rg.LoadModel(args.model)
	for i in range(10000):
		rg.GeneratePairs(1024)
		rg.OptimizeModel()
		Plot(rg.model.training_loss_container.Get(),rg.model.evaluation_loss_container.Get(),'loss',1,False)
