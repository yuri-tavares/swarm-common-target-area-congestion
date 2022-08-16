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

algorithmsLabels = ["PCC","EE","PCC-EE","TRVF","SQF"];
suffix_file_list = ['nonholo','holo']
nRobots = range(20,320,20);
datalines = 20+2

printValuesForTTest = False # Set true if one wishes to print values for t-test.
SetForTTest = ["SQF","EE","TRVF"] # Set of algorithms to print if the variable above is true.

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
        for option in range(datalines):
          if option == 20:
            FirstRobotReachingTimeLine = 8 
            LastRobotReachingTimeLine = 9
            data[n,a,s,option,i_sf] = (nRobots[n]-1)/((float(dataFileStr[LastRobotReachingTimeLine]) - float(dataFileStr[FirstRobotReachingTimeLine]))/1e6);
          elif option == 21:
            SimulationTimeLine = 10 
            LastRobotReachingTimeLine = 9
            data[n,a,s,option,i_sf] = (float(dataFileStr[SimulationTimeLine]) - float(dataFileStr[LastRobotReachingTimeLine]))/1e6;
          elif option == 19:
            if algorithmsLabels[a] in ['SQF','TRVF']:
              SumLeavingTimeLine = 19
            else:
              SumLeavingTimeLine = 13
            data[n,a,s,option,i_sf] = (float(dataFileStr[SumLeavingTimeLine])/1e6)/nRobots[n];
          elif option in [11,12] or (algorithmsLabels[a] in ['SQF','TRVF'] and option in [13,14,16,17]):
            data[n,a,s,option,i_sf] = float(dataFileStr[option]);
          elif option in [8,9,10]:
            data[n,a,s,option,i_sf] = float(dataFileStr[option])/1e6;
          elif option < len(dataFileStr):
            data[n,a,s,option,i_sf] = int(dataFileStr[option]);
          else:
            continue;
      for option in range(datalines):
        dataMean[n,a,option,i_sf] = np.mean(data[n,a,:nSamples[a],option,i_sf]);
        dataVari[n,a,option,i_sf] = np.var(data[n,a,:nSamples[a],option,i_sf]);
        if all(data[n,a,0,option,i_sf] == rest for rest in data[n,a,:,option,i_sf]):
          dataUpCi[n,a,option,i_sf] = dataMean[n,a,option,i_sf]
        else:
          dataUpCi[n,a,option,i_sf] = calcConfInt(dataMean[n,a,option,i_sf],dataVari[n,a,option,i_sf],nSamples[a]);

list_line_ylabel = [ 
#       Label                                                       # index
#-------------------------------------------------------------------#------
"Total number of iterations",                                       # 0
"Sum of the max. iterations",                                       # 1
"Number of messages",                                               # 2
"Summation of the iter. for reaching",                              # 3
"Summation of the iter. for exiting",                               # 4
"Last robot's iterations that\nreached the target",                 # 5
"Last robot's iterations that\nleft the target",                    # 6
"Stalls",                                                           # 7
"First robot's time for reaching\n the target region (s)",          # 8
"Last robot's reaching time (s)",                                   # 9
"Total time of the simulation (s)",                                 # 10
"Minimum distance (m)",                                             # 11
"Maximum velocity (m/s)",                                           # 12
"Mean distance (m)",                                                # 13
"St. dev. of distance (m)",                                         # 14
"Num. of measured distances",                                       # 15
"Mean velocity (m/s)",                                              # 16
"St. dev. of velocity (m/s)",                                       # 17
"Num. of measured velocities",                                      # 18
"Average leaving time (s)",                                         # 19
"Throughput (1/s)",                                                 # 20
"Last robot's time for leaving\nthe experiment area (s)"            # 21
]

plt.rcParams.update({'font.size': 15})

def mainLoop(option):
  if printValuesForTTest:
    print('==== '+list_line_ylabel[option]+' ====')
  for  i_sf, suffix_file in enumerate(suffix_file_list):
    algorithms = algorithmsDict[suffix_file] 
    algorithmsSymbol = algorithmsSymbolDict[suffix_file] 
    prefixNum = prefixNumDict[suffix_file] 
    for a in range(len(algorithmsLabels)):
      plt.errorbar(nRobots,dataMean[:,a,option,i_sf], yerr=[m - n for m,n in zip(dataUpCi[:,a,option,i_sf],dataMean[:,a,option,i_sf])], label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5,linestyle='solid' if algorithmsLabels[a] != 'TRVF' else 'dashed');
      if printValuesForTTest and algorithmsLabels[a] in SetForTTest:
        print('#',end='')
        print(algorithmsLabels[a],suffix_file,sep='-------')
        print('means'+algorithmsLabels[a]+' = ',end='')
        print(*dataMean[:,a,option,i_sf], sep=', ')
        print('vari'+algorithmsLabels[a]+' = ',end='')
        print(*dataVari[:,a,option,i_sf], sep=', ')
    plt.legend(loc=0);
    plt.xlabel("Number of robots");
    plt.ylabel(list_line_ylabel[option])
    plt.savefig("FigureRobots"+str(option)+suffix_file+".png",bbox_inches="tight",pad_inches=0.01);
    plt.savefig("FigureRobots"+str(option)+suffix_file+".pdf",bbox_inches="tight",pad_inches=0.01);
    plt.clf();


# ~ mainLoop(21)
# ~ mainLoop(20)
# ~ mainLoop(19)
mainLoop(10)
# ~ mainLoop(9)



