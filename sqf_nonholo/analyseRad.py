import sys
sys.path.insert(1, '../common/')
from hexagonal_packing_throughput import throughput, bestTheta
import numpy as np
import matplotlib.pyplot as plt
import math
import os
import subprocess
from scipy.stats import t
from array import array

def loadFile(filename):
  input_file = open(filename, 'rb')
  float_array = array('d')
  float_array.fromstring(input_file.read())
  return float_array

def saveFile(outfilename, out):
  output_file = open(outfilename, 'wb')
  float_array = array('d', out)
  float_array.tofile(output_file)
  output_file.close()

def calcConfInt(p):
  mean,var,size = np.mean(p),np.var(p),len(p)
  alpha = 0.01
  df = size - 1
  unbiased_sd = math.sqrt((size/df)*var)
  t_variate = t.ppf(1-alpha/2,df)
  uppervalue = mean + t_variate * unbiased_sd/math.sqrt(size)
  return uppervalue

algorithmsLabels = ["Experiments"];
algorithms = ['../target_topdown_influence']
algorithmsSymbol = ["."]*len(algorithmsLabels);
prefixNum=['rad']*len(algorithmsLabels);
rad = [0.4,0.5,0.6,0.7,0.8,0.9,1.0]
ksamples = 40
nSamples = np.full((len(rad),len(algorithms)), ksamples)
datalines = 13+1
data = np.zeros((len(rad),len(algorithms),ksamples,datalines));
dataMean = np.zeros((len(rad),len(algorithms),datalines));
dataVari = np.zeros((len(rad),len(algorithms),datalines));
dataUpCi = np.zeros((len(rad),len(algorithms),datalines));
nNotAllowed = np.zeros((len(rad),len(algorithms)))

#Count how many logs has maximum allowed time
for a in range(len(algorithms)):
  for n in range(len(rad)):
    algorithm = algorithms[a];
    for s in range(nSamples[n,a]):
      dataFile = open(algorithm+"/"+prefixNum[a]+('%.1f' % rad[n])+"/log_"+str(s));
      dataFileStr = dataFile.readlines();
      if int(dataFileStr[10]) >= 20*60*1000000:
        nNotAllowed[n,a] += 1;

for a in range(len(algorithms)):
  for n in range(len(rad)):
    algorithm = algorithms[a];
    index_list = []
    for s in range(nSamples[n,a]):
      dataFile = open(algorithm+"/"+prefixNum[a]+('%.1f' % rad[n])+"/log_"+str(s));
      dataFileStr = dataFile.readlines();
      if int(dataFileStr[10]) >= 20*60*1000000: continue
      index_list.append(s)
      for fileline in range(datalines):
        if fileline == 13:
          FirstRobotReachingTimeline = 8 
          LastRobotReachingTimeline = 9
          # number of robots is 100
          data[n, a, s, fileline] = 99/((float(dataFileStr[LastRobotReachingTimeline]) - float(dataFileStr[FirstRobotReachingTimeline]))/1e6);
        elif fileline in [11,12]:
          data[n, a, s, fileline] = float(dataFileStr[fileline]);
        elif fileline in [8,9,10]:
          data[n, a, s, fileline] = float(dataFileStr[fileline])/1e6;
        else:
          data[n, a, s, fileline] = int(dataFileStr[fileline]);
    if index_list != []:
      for fileline in range(datalines):
        tmp_data = [data[n,a,s,fileline] for s in index_list]
        dataMean[n, a, fileline] = np.mean(tmp_data);
        if all(dataMean[n, a, fileline] == rest for rest in tmp_data):
          dataUpCi[n, a, fileline] = dataMean[n, a, fileline]
        else:
          dataUpCi[n, a, fileline] = calcConfInt(tmp_data);

def fhmin(s,d,v):
  return 4*v*s/(math.sqrt(3)*d*d) - 2*v/(math.sqrt(3)*d)

def f(s,v,d):
  if 0 < s and s <= math.sqrt(3)*d/4.:
    return v/(d*math.sqrt(1-((2.0*s)/d)**2))
  elif s < d/2.:
    return 2*v/d
  else:
    return fhmin(s,d,v);

def plotTheoreticalThroughput(rads,a,my_marker,axes):
  maxVelocityLine = 12
  minDistanceLine = 11
  vs = [max(data[n, a, :nSamples[n,a], maxVelocityLine]) for n in range(len(rads))]
  ds = [min(data[n, a, :nSamples[n,a], minDistanceLine]) for n in range(len(rads))]
  fs = [f(rads[i],vs[i],ds[i]) for i in range(len(rads))]
  axes.plot(rads,fs,color='tab:orange',marker=my_marker)

def mainLoop(fileline):
  # ~ plt.figure(figsize=(7.5,4.5));
  plt.rcParams.update({'font.size': 15})
  for a in range(len(algorithms)):
    allowedRad = []
    for n in range(len(rad)):
      if nSamples[n,a] - nNotAllowed[n,a] > 1:
        allowedRad.append(n)
    if allowedRad != []:
      tmpRad = [rad[i] for i in allowedRad]
      tmpDataMean = [dataMean[n,a,fileline] for n in allowedRad]
      tmpyErr = [dataUpCi[n,a,fileline] - dataMean[n,a,fileline] for n in allowedRad]
    if fileline != 13:
      plt.errorbar(tmpRad,tmpDataMean, yerr=tmpyErr, label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5,linestyle='solid' if algorithmsLabels[a] != 'SQF' else 'dashed');
    else:
      fig, (ax1, ax2) = plt.subplots(2, 1)
      ax2.errorbar(tmpRad,tmpDataMean, yerr=tmpyErr, label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5,linestyle='solid' if algorithmsLabels[a] != 'SQF' else 'dashed');
      ax2.set_title("Experiments")
      plotTheoreticalThroughput(tmpRad,a,algorithmsSymbol[a],ax1)
      ax1.set_title("Asymptotic")
      for ax in fig.get_axes():
        ax.label_outer()
  if fileline != 13:
    plt.legend(loc=0);
  plt.xlabel("Number of robots");
  list_line_ylabel = [ 
    #       Label                                  # index
    #--------------------------------------------------#-------
    "Total number of iterations",                      # 0
    "Total iterations of the last robot",              # 1
    "Number of messages",                              # 2
    "Summation of the iter. for reaching",             # 3
    "Summation of the iter. for exiting",              # 4
    "Last robot's iterations for reaching",            # 5
    "Last robot's iterations for exiting",             # 6
    "Stalls",                                          # 7
    "Time for reaching target of the first robot (s)", # 8
    "Time for reaching target of the last robot (s)",  # 9
    "Total time of the simulation (s)",                # 10
    "Minimum distance (m)",                            # 11
    "Maximum velocity (m/s)",                          # 12
    "Throughput (1/s)"                                 # 13
  ]
  # ~ print(list_line_ylabel[fileline])
  # ~ print(testTEqual(dataMean[:,0,fileline], dataVari[:,0,fileline], nSamples[0], dataMean[:,1,fileline], dataVari[:,1,fileline], nSamples[1]))
  plt.ylabel(list_line_ylabel[fileline])
  if fileline != 13:
    plt.savefig("FigureRobots"+str(fileline)+".pdf",bbox_inches="tight",pad_inches=0.01);
    plt.savefig("FigureRobots"+str(fileline)+".png",bbox_inches="tight",pad_inches=0.01);
  else:
    plt.savefig("SQF-throughput-th-exp-nonholo.pdf",bbox_inches="tight",pad_inches=0.01);
    plt.savefig("SQF-throughput-th-exp-nonholo.png",bbox_inches="tight",pad_inches=0.01);
  plt.clf();


mainLoop(13)

