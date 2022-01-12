import sys
import numpy as np
import matplotlib.pyplot as plt
import math
import os
import subprocess
from scipy.stats import t
from array import array

def calcConfInt(mean,var,size):
  alpha = 0.01
  df = size - 1
  unbiased_sd = math.sqrt((size/df)*var)
  t_variate = t.ppf(1-alpha/2,df)
  uppervalue = mean + t_variate * unbiased_sd/math.sqrt(size)
  return uppervalue

def testTEqual(means1, variances1, n1, means2, variances2, n2):
  '''
    means1, means2: vector
    variances1, variances2: vector
    n1,n2: integer
    return True if p-value >= 0.05, that is, means are equal.
  '''
  l1,l2 = len(means1),len(means2)
  df = n1 + n2 - 2
  ts = [(means1[i] - means2[i])/math.sqrt(variances1[i]/n1 + variances2[i]/n2) for i in range(min(l1,l2))]
  ps = [(1-t.cdf(abs(ts[i]),df))*2 for i in range(min(l1,l2))]
  return [(ps[i] >= 0.05, ps[i]) for i in range(min(l1,l2))]


algorithmsLabels = ["PCC","EE","PCC-EE","TRVF","SQF"];
suffix_file_list = ['nonholo','holo']
nRobots = range(20,320,20);
datalines = 13+3

algorithmsDict = {}
algorithmsSymbolDict = {}
prefixNumDict = {}

suffix_file = 'nonholo'
algorithmsDict[suffix_file] = ['../target_pcc','../target_ee','../target_pcc-ee','../cool path vector field/','../target_topdown_influence']
algorithmsSymbolDict[suffix_file] = ["."]*len(algorithmsLabels);
prefixNumDict[suffix_file] = ['nRobos']*3 + ["nonholo/K_5/n_"] + ['nRobos']

suffix_file = 'holo'
algorithmsDict[suffix_file] = ['../target_holonomic_pcc','../target_holonomic_ee','../target_holonomic_pcc-ee','../cool path vector field/','../target_holonomic_topdown_influence']
algorithmsSymbolDict[suffix_file] = ["."]*len(algorithmsLabels);
prefixNumDict[suffix_file] = ['nRobos']*3 + ["holo/K_5/n_"] + ['nRobos']

nSamples = [40]*len(algorithmsLabels);
data = np.zeros((len(nRobots),len(algorithmsLabels),max(nSamples),datalines,len(suffix_file_list)));
dataMean = np.zeros((len(nRobots),len(algorithmsLabels),datalines,len(suffix_file_list)));
dataVari = np.zeros((len(nRobots),len(algorithmsLabels),datalines,len(suffix_file_list)));
dataUpCi = np.zeros((len(nRobots),len(algorithmsLabels),datalines,len(suffix_file_list)));

for i_sf, suffix_file in enumerate(suffix_file_list):
  algorithms = algorithmsDict[suffix_file] 
  algorithmsSymbol = algorithmsSymbolDict[suffix_file] 
  prefixNum = prefixNumDict[suffix_file] 
  for n in range(len(nRobots)):
    for a in range(len(algorithmsLabels)):
      algorithm = algorithms[a];
      for s in range(nSamples[a]):
        dataFile = open(algorithm+"/"+prefixNum[a]+str(nRobots[n])+"/log_"+str(s));
        dataFileStr = dataFile.readlines();
        for fileline in range(datalines):
          if fileline == 13:
            FirstRobotReachingTimeLine = 8 
            LastRobotReachingTimeLine = 9
            data[n,a,s,fileline,i_sf] = (nRobots[n]-1)/((float(dataFileStr[LastRobotReachingTimeLine]) - float(dataFileStr[FirstRobotReachingTimeLine]))/1e6);
          elif fileline == 14:
            SimulationTimeLine = 10 
            LastRobotReachingTimeLine = 9
            data[n,a,s,fileline,i_sf] = (float(dataFileStr[SimulationTimeLine]) - float(dataFileStr[LastRobotReachingTimeLine]))/1e6;
          elif fileline in [11,12]:
            data[n,a,s,fileline,i_sf] = float(dataFileStr[fileline]);
          elif fileline in [8,9,10]:
            data[n,a,s,fileline,i_sf] = float(dataFileStr[fileline])/1e6;
          elif fileline == 15:
            SumExitingIterationsLine = 4
            data[n,a,s,fileline,i_sf] = float(dataFileStr[SumExitingIterationsLine])/nRobots[n];
          else:
            data[n,a,s,fileline,i_sf] = int(dataFileStr[fileline]);
      for fileline in range(datalines):
        dataMean[n,a,fileline,i_sf] = np.mean(data[n,a,:nSamples[a],fileline,i_sf]);
        dataVari[n,a,fileline,i_sf] = np.var(data[n,a,:nSamples[a],fileline,i_sf]);
        if all(data[n,a,0,fileline,i_sf] == rest for rest in data[n,a,:,fileline,i_sf]):
          dataUpCi[n,a,fileline,i_sf] = dataMean[n,a,fileline,i_sf]
        else:
          dataUpCi[n,a,fileline,i_sf] = calcConfInt(dataMean[n,a,fileline,i_sf],dataVari[n,a,fileline,i_sf],nSamples[a]);


plt.rcParams.update({'font.size': 15})

def mainLoop(fileline):
  for  i_sf, suffix_file in enumerate(suffix_file_list):
    algorithms = algorithmsDict[suffix_file] 
    algorithmsSymbol = algorithmsSymbolDict[suffix_file] 
    prefixNum = prefixNumDict[suffix_file] 
    for a in range(len(algorithmsLabels)):
      plt.errorbar(nRobots,dataMean[:,a,fileline,i_sf], yerr=[m - n for m,n in zip(dataUpCi[:,a,fileline,i_sf],dataMean[:,a,fileline,i_sf])], label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5,linestyle='solid' if algorithmsLabels[a] != 'TRVF' else 'dashed');
      if algorithmsLabels[a] in  ["SQF","EE","TRVF"]:
        print('#',end='')
        print(algorithmsLabels[a],suffix_file,sep='-------')
        print('means'+algorithmsLabels[a]+' = ',end='')
        print(*dataMean[:,a,fileline,i_sf], sep=', ')
        print('vari'+algorithmsLabels[a]+' = ',end='')
        print(*dataVari[:,a,fileline,i_sf], sep=', ')

    if suffix_file == "nonholo" and fileline == 13:
      plt.legend(loc="lower center");
    else:
      plt.legend(loc=0);
    plt.xlabel("Number of robots");
    list_line_ylabel = [ 
      #       Label                                            # index
      #----------------------------------------------------------#------
      "Total number of iterations",                              # 0
      "Total iterations of the last robot",                      # 1
      "Number of messages",                                      # 2
      "Summation of the iter. for reaching",                     # 3
      "Summation of the iter. for exiting",                      # 4
      "Last robot's iterations for reaching",                    # 5
      "Last robot's iterations for exiting",                     # 6
      "Stalls",                                                  # 7
      "First robot's time for reaching\n the target region (s)", # 8
      "Last robot's time for reaching\n the target region (s)",  # 9
      "Total time of the simulation (s)",                        # 10
      "Minimum distance (m)",                                    # 11
      "Maximum velocity (m/s)",                                  # 12
      "Throughput (1/s)",                                        # 13
      "Last robot's time for leaving\n the experiment area (s)", # 14
      "Sum of the iter. for exiting by number of robots",        # 15
    ]
    plt.ylabel(list_line_ylabel[fileline])
    plt.savefig("FigureRobots"+str(fileline)+suffix_file+".png",bbox_inches="tight",pad_inches=0.01);
    plt.savefig("FigureRobots"+str(fileline)+suffix_file+".pdf",bbox_inches="tight",pad_inches=0.01);
    plt.clf();

mainLoop(15)
mainLoop(6)
mainLoop(4)
# ~ mainLoop(9)
# ~ mainLoop(13)
# ~ mainLoop(10)
# ~ mainLoop(14)

