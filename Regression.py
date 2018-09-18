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
use_cuda = torch.cuda.is_available()
FloatTensor = torch.cuda.FloatTensor if use_cuda else torch.FloatTensor
LongTensor = torch.cuda.LongTensor if use_cuda else torch.LongTensor
ByteTensor = torch.cuda.ByteTensor if use_cuda else torch.ByteTensor
Tensor = FloatTensor

Tuple = namedtuple('Tuple',('s','qdd_des','a','A','b','q','q_d','v','v_d'))
class Buffer(object):
	def __init__(self,buff_size = 65536):
		super(Buffer, self).__init__()
		self.buff_size = buff_size
		self.buffer = deque(maxlen = buff_size)

	def Push(self,*args):
		self.buffer.append(Tuple(*args))

	def Clear(self):
		self.buffer.clear()
	def Full(self):
		if len(self.buffer)==self.buff_size:
			return True
		else:
			return False;
class Regressor(object):
	def __init__(self,num_slaves):
		np.random.seed(seed = int(time.time()))
		self.env = Preprocess(num_slaves)
		self.num_slaves = num_slaves
		
		self.num_states = self.env.GetNumStates()
		self.num_dofs = self.env.GetNumDofs()
		self.num_muscles = self.env.GetNumMuscles()
		self.num_input = self.num_states + self.num_dofs

		self.num_minibatch = 128
		self.num_evaluation = 0

		self.model = MuscleNN(self.num_input,self.num_muscles)
		if use_cuda:
			self.model.cuda()
		self.normalizer = Tensor(2,self.num_input)
		self.normalizer[1][0:self.num_states] = 1.0
		self.normalizer[1][self.num_states:] = 1000.0
		self.model.SetNormalizer(self.normalizer[0][0:self.num_input],self.normalizer[1][0:self.num_input])
		
		self.optimizer = optim.Adam(self.model.parameters(),lr=1E-5)

		self.loss = 0.0
		self.buffer = Buffer(65536*2)
		self.tic = time.time()
	def GetTuple(self,num):
		arr = self.buffer.buffer[num%len(self.buffer.buffer)]
		return [arr.s, arr.qdd_des, arr.a, arr.A, arr.b, arr.q, arr.q_d, arr.v, arr.v_d]
	def SaveModel(self):
		self.model.save('../nn_muscle/'+str(self.num_evaluation)+'.pt')

	def LoadModel(self,model_path):
		self.model.load(model_path)
		self.num_evaluation = int(self.model.training_loss_container.counter.data[0].cpu().numpy().tolist())

	def GenerateTuples(self,num):
		self.env.GenerateTuples(num)
		tuples = self.env.Get()
		for j in range(len(tuples)):
			self.buffer.Push(tuples[j][0],tuples[j][1],tuples[j][2],tuples[j][3],tuples[j][4],tuples[j][5],tuples[j][6],tuples[j][7],tuples[j][8])

	def OptimizeModel(self):
		all_tuples = np.array(self.buffer.buffer)
		np.random.shuffle(all_tuples)

		for i in range(len(all_tuples)//self.num_minibatch):
			tuples = all_tuples[i*self.num_minibatch:(i+1)*self.num_minibatch]
			batch = Tuple(*zip(*tuples))
			
			stack_s = np.vstack(batch.s).astype(np.float32)
			stack_qdd_des = np.vstack(batch.qdd_des).astype(np.float32)
			
			stack_a = np.vstack(batch.a).astype(np.float32)
			stack_A = np.vstack(batch.A).astype(np.float32)
			stack_A = stack_A.reshape(self.num_minibatch,self.num_dofs,self.num_muscles)
			stack_b = np.vstack(batch.b).astype(np.float32)
			
			stack_input = np.concatenate([stack_s,stack_qdd_des],axis=1)
			stack_input = Tensor(stack_input)
			stack_a = Tensor(stack_a)
			stack_qdd_des = Tensor(stack_qdd_des)
			stack_A = Tensor(stack_A)
			stack_b = Tensor(stack_b)
			
			a_inference = self.model(stack_input)
			qdd_inference = torch.einsum('ijk,ik->ij',(stack_A,a_inference)) + stack_b
			loss = 0.01*((a_inference).pow(2)).mean() + (((stack_qdd_des-qdd_inference)/self.normalizer[1][self.num_states:]).pow(2).mean())

			self.loss = loss.data.tolist()
				
			self.optimizer.zero_grad()
			loss.backward(retain_graph=True)
			for param in self.model.parameters():
				if param.grad is not None:
					param.grad.data.clamp_(-0.5,0.5)
			self.optimizer.step()
		self.model.training_loss_container.Push(self.loss)
		self.num_evaluation += 1
		self.SaveModel()


import matplotlib
import matplotlib.pyplot as plt
plt.ion()

# def Plot(y,y1,title,num_fig=1,ylim=True):
# 	plt.figure(num_fig)
# 	plt.clf()
	
# 	plt.title(title)
# 	plt.hold(True)	
# 	if(len(y)>100):
# 		plt.plot(y[len(y)-100:],'b')
# 		plt.plot(y1[len(y1)-100:],'r')
# 	else:
# 		plt.plot(y,'b')
# 		plt.plot(y1,'r')
# 	plt.show()
# 	if ylim:
# 		plt.ylim([0,12])
# 	plt.pause(0.001)

def Plot(y,title,num_fig=1,ylim=True):
	plt.figure(num_fig)
	plt.clf()
	
	plt.title(title)
	plt.hold(True)	
	if(len(y)>100):
		plt.plot(y[len(y)-100:],'b')
	else:
		plt.plot(y,'b')
	plt.show()
	if ylim:
		plt.ylim([0,12])
	plt.pause(0.001)
import argparse

if __name__=="__main__":
	print(use_cuda)
	rg = Regressor(16)

	parser = argparse.ArgumentParser()
	parser.add_argument('-m','--model',help='actor model directory')
	args =parser.parse_args()
	if args.model is not None:
		rg.LoadModel(args.model)
	for i in range(100000):
		if i%500 == 0:
			rg.buffer.Clear()
		if i%10 == 0 and rg.buffer.Full() is not True:
			rg.GenerateTuples(1024)
		
		rg.OptimizeModel()
		# Plot(rg.model.training_loss_container.Get(),rg.model.evaluation_loss_container.Get(),'loss',1,False)
		Plot(rg.model.training_loss_container.Get(),'loss',1,False)
