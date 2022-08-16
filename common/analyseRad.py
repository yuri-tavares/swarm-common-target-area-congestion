import sys
import numpy as np
import matplotlib.pyplot as plt
import math
import os
import subprocess
from scipy.stats import t

def calcConfInt(p):
  mean,var,size = np.mean(p),np.var(p),len(p)
  alpha = 0.01
  df = size - 1
  unbiased_sd = math.sqrt((size/df)*var)
  t_variate = t.ppf(1-alpha/2,df)
  uppervalue = mean + t_variate * unbiased_sd/math.sqrt(size)
  return uppervalue


algorithmsLabels = ["EE","PCC-EE","PCC","SQF"];
suffix_file_list = ['nonholo','holo']
rad = [0.4,0.5,0.6,0.7,0.8,0.9,1.0]
logLines = 13+1
# list to ignore when plot not allowed log graph.
ignoreLabels = []

algorithmsDict = {}
algorithmsSymbolDict = {}
prefixNumDict = {}

suffix_file = 'nonholo'
algorithmsDict[suffix_file] = ['../target_ee','../target_pcc-ee','../target_pcc','../target_topdown_influence']
algorithmsSymbolDict[suffix_file] = ["."]*len(algorithmsLabels)
prefixNumDict[suffix_file] = ['rad']*len(algorithmsLabels)

suffix_file = 'holo'
algorithmsDict[suffix_file] = ['../target_holonomic_ee','../target_holonomic_pcc-ee','../target_holonomic_pcc','../target_holonomic_topdown_influence']
algorithmsSymbolDict[suffix_file] = ["."]*len(algorithmsLabels)
prefixNumDict[suffix_file] = ['rad']*len(algorithmsLabels)

ksamples = 40
nSamples = np.full((len(rad),len(algorithmsLabels),len(suffix_file_list)), ksamples)
data = np.zeros((len(rad),len(algorithmsLabels),ksamples,logLines,len(suffix_file_list)));
dataMean = np.zeros((len(rad),len(algorithmsLabels),logLines,len(suffix_file_list)));
dataUpCi = np.zeros((len(rad),len(algorithmsLabels),logLines,len(suffix_file_list)));
nNotAllowed = np.zeros((len(rad),len(algorithmsLabels),len(suffix_file_list)))

#Count how many logs has maximum allowed time
for i_sf, suffix_file in enumerate(suffix_file_list):
  algorithms = algorithmsDict[suffix_file] 
  prefixNum = prefixNumDict[suffix_file] 
  for a in range(len(algorithmsLabels)):
    for n in range(len(rad)):
      algorithm = algorithms[a];
      for s in range(nSamples[n,a,i_sf]):
        dataFile = open(algorithm+"/"+prefixNum[a]+('%.1f' % rad[n])+"/log_"+str(s));
        dataFileStr = dataFile.readlines();
        if int(dataFileStr[10]) >= 20*60*1000000:
          nNotAllowed[n,a,i_sf] += 1;

printNumberLogs = False


def plotBar(i,W,N,Xms,Ys,my_label):
  '''
  Plot the i-th bars for a alternate bars graph.
    i: i-th bar to plot;
    W: width of all bars together;
    N: total number of bars;
    Xms: middle x-axis coordinate of all bars together;
    Ys: list of y-axis coordinate values of the i-th bar;
    my_label: the label for the i-th bar.
  '''
  pos = [Xm + (W*(2*i-N+1))/(2*N) for Xm in Xms]
  plt.bar(pos,Ys,width=W/N,label=my_label)

def plotNumber(i,W,N,Xms,Ys):
  '''
  Plot the number in Ys as text above the i-th bar.
    i: i-th bar to plot;
    W: width of all bars together;
    N: total number of bars;
    Xms: middle x-axis coordinate of all bars together;
    Ys: list of y-axis coordinate values of the i-th bar;
  '''
  pos = [Xm + W*(2*i-N+1)/(2*N) for Xm in Xms]
  for i in range(len(Ys)):
    plt.annotate(str(int(Ys[i])),(pos[i],Ys[i]),ha='center')

plt.rcParams.update({'font.size': 15})

def imgAllowed():
  #Save a figure showing the number of logs with maximum allowed time
  for i_sf, suffix_file in enumerate(suffix_file_list):
    for a in range(len(algorithmsLabels)):
      if not (algorithmsLabels[a] in ignoreLabels):
        plotBar(a,(1-0.10)*0.1,len(algorithmsLabels),rad,nNotAllowed[:len(rad),a,i_sf],algorithmsLabels[a])
        if printNumberLogs:
          plotNumber(a,(1-0.10)*0.1,len(algorithmsLabels),rad,nNotAllowed[:len(rad),a,i_sf])
    plt.xticks(rad)
    if len(algorithmsLabels) > 1:
      plt.legend(loc='center right');
    plt.ylabel("Number of fails");
    plt.xlabel("Radius of target area");
    plt.savefig("FigureNotAllowedLogs"+suffix_file+".png",bbox_inches="tight");
    plt.savefig("FigureNotAllowedLogs"+suffix_file+".pdf",bbox_inches="tight");
    plt.clf();
  

for i_sf, suffix_file in enumerate(suffix_file_list):
  algorithms = algorithmsDict[suffix_file] 
  prefixNum = prefixNumDict[suffix_file] 
  for a in range(len(algorithmsLabels)):
    for n in range(len(rad)):
      algorithm = algorithms[a];
      index_list = []
      for s in range(nSamples[n,a,i_sf]):
        dataFile = open(algorithm+"/"+prefixNum[a]+('%.1f' % rad[n])+"/log_"+str(s));
        dataFileStr = dataFile.readlines();
        if int(dataFileStr[10]) >= 20*60*1000000: continue
        index_list.append(s)
        for fileline in range(logLines):
          if fileline == 13:
            FirstRobotReachingTimeline = 8 
            LastRobotReachingTimeline = 9
            # number of robots is 100
            data[n,a,s,fileline,i_sf] = 99/((float(dataFileStr[LastRobotReachingTimeline]) - float(dataFileStr[FirstRobotReachingTimeline]))/1e6);
          elif fileline in [11,12]:
            data[n,a,s,fileline,i_sf] = float(dataFileStr[fileline]);
          elif fileline in [8,9,10]:
            data[n,a,s,fileline,i_sf] = float(dataFileStr[fileline])/1e6;
          else:
            data[n, a, s, fileline,i_sf] = int(dataFileStr[fileline]);
      if index_list != []:
        for fileline in range(logLines):
          tmp_data = [data[n,a,s,fileline,i_sf] for s in index_list]
          dataMean[n,a,fileline,i_sf] = np.mean(tmp_data);
          if all(dataMean[n,a,fileline,i_sf] == rest for rest in tmp_data):
            dataUpCi[n,a,fileline,i_sf] = dataMean[n,a,fileline,i_sf]
          else:
            dataUpCi[n,a,fileline,i_sf] = calcConfInt(tmp_data);

def main(fileline):
  for i_sf, suffix_file in enumerate(suffix_file_list):
    algorithmsSymbol = algorithmsSymbolDict[suffix_file] 
    for a in range(len(algorithmsLabels)):
      allowedRad = []
      for n in range(len(rad)):
        if nSamples[n,a,i_sf] - nNotAllowed[n,a,i_sf] > 1:
          allowedRad.append(n)
      if allowedRad != []:
        tmpRad = [rad[i] for i in allowedRad]
        tmpDataMean = [dataMean[n,a,fileline,i_sf] for n in allowedRad]
        tmpyErr = [dataUpCi[n,a,fileline,i_sf] - dataMean[n,a,fileline,i_sf] for n in allowedRad]
        plt.errorbar(tmpRad,tmpDataMean, yerr=tmpyErr, label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5,linestyle='solid' if algorithmsLabels[a] != 'SQF' else 'dashed');
        if printNumberLogs:
          valuesStr = [str(int(nSamples[n,a,i_sf] - nNotAllowed[n,a,i_sf])) for n in allowedRad]
          for i in range(len(valuesStr)):
            plt.annotate(valuesStr[i],(tmpRad[i],tmpDataMean[i]))
    if len(algorithmsLabels) > 1:
      plt.legend(loc=0);
    plt.xlabel("Radius of target area (m)");
    list_line_ylabel = [ 
      #       Label                                              # index
      #----------------------------------------------------------#------
      "Total number of iterations",                              # 0
      "Total iterations of the last robot",                      # 1
      "Number of messages",                                      # 2
      "Summation of the iter. for reaching",                     # 3
      "Summation of the iter. for exiting",                      # 4
      "Last robot's iterations for reaching",                    # 5
      "Last robot's iterations for exiting",                     # 6
      "Stalls",                                                  # 7
      "First robot's reaching time (s)",                         # 8
      "Last robot's reaching time (s)",                          # 9
      "Total time of the simulation (s)",                        # 10
      "Minimum distance (m)",                                    # 11
      "Maximum velocity (m/s)",                                  # 12
      "Throughput (1/s)"                                         # 13
    ]
    plt.ylabel(list_line_ylabel[fileline])
    plt.savefig("FigureRad"+str(fileline)+suffix_file+".png",bbox_inches="tight");
    plt.savefig("FigureRad"+str(fileline)+suffix_file+".pdf",bbox_inches="tight");
    plt.clf();



imgAllowed()
main(9)
# ~ main(10)
