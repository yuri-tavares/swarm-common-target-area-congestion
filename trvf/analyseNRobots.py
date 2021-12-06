import sys
sys.path.insert(1, '../common/')
from cool_path_throughput import limitF, Kmax
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
  float_array.frombytes(input_file.read())
  return float_array

def saveFile(outfilename, out):
  output_file = open(outfilename, 'wb')
  float_array = array('d', out)
  float_array.tofile(output_file)
  output_file.close()

def calcConfInt(mean,var,size):
  alpha = 0.01
  df = size - 1
  unbiased_sd = math.sqrt((size/df)*var)
  t_variate = t.ppf(1-alpha/2,df)
  uppervalue = mean + t_variate * unbiased_sd/math.sqrt(size)
  return uppervalue

algorithms = ['./holo/K_5/','./nonholo/K_5/']
algorithmsLabels = ["Experiments"]*len(algorithms);
algorithmsSymbol = ["."]*len(algorithmsLabels);
prefixNum='n_'
nRobots = range(20, 320, 20)
nSamples = [40]*len(algorithmsLabels);
datalines = 19+1
data = np.zeros((len(nRobots),len(algorithms),max(nSamples),datalines));
dataMean = np.zeros((len(nRobots),len(algorithms),datalines));
dataVari = np.zeros((len(nRobots),len(algorithms),datalines));
dataUpCi = np.zeros((len(nRobots),len(algorithms),datalines));


for n in range(len(nRobots)):
  for a in range(len(algorithms)):
    algorithm = algorithms[a];
    for s in range(nSamples[a]):
      dataFile = open(algorithm+"/"+prefixNum+str(nRobots[n])+"/log_"+str(s));
      dataFileStr = dataFile.readlines();
      for fileline in range(datalines):
        if fileline == 19:
          FirstRobotReachingTimeline = 8 
          LastRobotReachingTimeline = 9
          data[n, a, s, fileline] = (nRobots[n]-1)/((float(dataFileStr[LastRobotReachingTimeline]) - float(dataFileStr[FirstRobotReachingTimeline]))/1e6);
        elif fileline in [11,12,13,14,16,17]:
          data[n, a, s, fileline] = float(dataFileStr[fileline]);
        else:
          data[n, a, s, fileline] = int(dataFileStr[fileline]);
    for fileline in range(datalines):
      dataMean[n, a, fileline] = np.mean(data[n,a,:nSamples[a],fileline]);
      dataVari[n, a, fileline] = np.var(data[n,a,:nSamples[a],fileline]);
      if all(data[n,a,0,fileline] == rest for rest in data[n,a,:,fileline]):
        dataUpCi[n, a, fileline] = dataMean[n, a, fileline]
      else:
        dataUpCi[n, a, fileline] = calcConfInt(dataMean[n, a, fileline],dataVari[n, a, fileline],nSamples[a]);

def testTEqual(means1, variances1, n1, means2, variances2, n2):
  '''
    means1, means2: vector
    variances1, variances2: vector
    n1,n2: integer
  '''
  l1,l2 = len(means1),len(means2)
  df = min(n1,n2) - 1
  ts = [(means1[i] - means2[i])/math.sqrt(variances1[i]/n1 + variances2[i]/n2) for i in range(min(l1,l2))]
  ps = [t.cdf(ts[i],df) for i in range(min(l1,l2))]
  return [ps[i] >= 0.05 for i in range(min(l1,l2))]

def iterativeMeanStDev(ms,stds,samps):
  '''
    From parallelly calculated vectors of means and standard deviations, and a vector of the number of samples used to calculate them, it returns the mean and standard deviation of the complete data.
    Arguments:
      ms, stds, samps: vectors of standard deviations and means.
    Returns mean and standard deviation.
  '''
  var = [s*s for s in stds]
  n2 = v2 = m2 = 0
  for i in range(len(ms)):
    v2 = (n2*v2 + samps[i]*var[i] + ((m2 - ms[i])*(m2 - ms[i])*n2*samps[i])/(n2 + samps[i]))/(n2 + samps[i]);
    m2 = (m2*n2 + ms[i]*samps[i])/(n2 + samps[i]);
    n2 += samps[i];
  return (m2,math.sqrt(v2))

def plotTheoreticalMeanLimits(nRobots,a,my_marker,my_label,axes=None):
  meanDistanceLine = 13
  stDevDistanceLine = 14
  samplesDistanceLine = 15
  meanVelocityLine = 16
  stDevVelocityLine = 17
  samplesVelocityLine = 18
  radius_s, K_value = 3, 5
  meansD, stdsD = zip(*[iterativeMeanStDev(data[n, a, :nSamples[a], meanDistanceLine],data[n, a, :nSamples[a], stDevDistanceLine],data[n, a, :nSamples[a], samplesDistanceLine]) for n in range(len(nRobots))])
  lowerD, upperD = [meansD[i] - stdsD[i] for i in range(len(nRobots))], [meansD[i] + stdsD[i] for i in range(len(nRobots))]
  meansV, stdsV = zip(*[iterativeMeanStDev(data[n, a, :nSamples[a], meanVelocityLine],data[n, a, :nSamples[a], stDevVelocityLine],data[n, a, :nSamples[a], samplesVelocityLine]) for n in range(len(nRobots))])
  lowerV, upperV = [meansV[i] - stdsV[i] for i in range(len(nRobots))],[meansV[i] + stdsV[i] for i in range(len(nRobots))]
  lowerLim, upperLim = [limitF(K_value,radius_s,lowerV[i],upperD[i]) for i in range(len(nRobots))], [limitF(K_value,radius_s,upperV[i],lowerD[i]) for i in range(len(nRobots))]
  middleLim, errLim = [(upperLim[i] + lowerLim[i])/2 for i in range(len(nRobots))], [(upperLim[i] - lowerLim[i])/2 for i in range(len(nRobots))]
  if axes != None:
    axes.errorbar(nRobots, middleLim, errLim, color='tab:orange', label=my_label, marker=my_marker, capsize=5);
  else:
    plt.errorbar(nRobots, middleLim, errLim, color='tab:orange', label=my_label, marker=my_marker, capsize=5);

def plotMeanLimitsFromHistogram(nRobots,a,my_marker,my_label,axes):
  meanDistanceLine = 13
  stDevDistanceLine = 14
  samplesDistanceLine = 15
  meanVelocityLine = 16
  stDevVelocityLine = 17
  samplesVelocityLine = 18
  radius_s, K_value = 3, 5
  holoSubStr = "nonholo" if "nonholo" in algorithms[a] else "holo"
  sufx = "_d"
  counts = [loadFile("bin/countsTRVF-"+sufx+"-"+str(nRobots[n])+"-"+holoSubStr+".bin") for n in range(len(nRobots))]
  bins = [np.array(loadFile("bin/binsTRVF-"+sufx+"-"+str(nRobots[n])+"-"+holoSubStr+".bin")) for n in range(len(nRobots))]
  mids = [0.5*(bins[n][1:] + bins[n][:-1]) for n in range(len(nRobots))]
  meansD = [np.average(mids[n], weights=counts[n]) for n in range(len(nRobots))]
  stdsD = [math.sqrt(np.average((mids[n] - meansD[n])**2, weights=counts[n])) for n in range(len(nRobots))]
  lowerD, upperD = [meansD[i] - stdsD[i] for i in range(len(nRobots))], [meansD[i] + stdsD[i] for i in range(len(nRobots))]
  sufx = "_v"
  counts = [loadFile("bin/countsTRVF-"+sufx+"-"+str(nRobots[n])+"-"+holoSubStr+".bin") for n in range(len(nRobots))]
  bins = [np.array(loadFile("bin/binsTRVF-"+sufx+"-"+str(nRobots[n])+"-"+holoSubStr+".bin")) for n in range(len(nRobots))]
  mids = [0.5*(bins[n][1:] + bins[n][:-1]) for n in range(len(nRobots))]
  meansV = [np.average(mids[n], weights=counts[n]) for n in range(len(nRobots))]
  stdsV = [math.sqrt(np.average((mids[n] - meansV[n])**2, weights=counts[n])) for n in range(len(nRobots))]
  lowerV, upperV = [meansV[i] - stdsV[i] for i in range(len(nRobots))], [meansV[i] + stdsV[i] for i in range(len(nRobots))]
  lowerLim, upperLim = [limitF(K_value,radius_s,lowerV[i],upperD[i]) for i in range(len(nRobots))], [limitF(K_value,radius_s,upperV[i],lowerD[i]) for i in range(len(nRobots))]
  middleLim, errLim = [(upperLim[i] + lowerLim[i])/2 for i in range(len(nRobots))], [(upperLim[i] - lowerLim[i])/2 for i in range(len(nRobots))]
  axes.errorbar(nRobots, middleLim, errLim, color='tab:green', label=my_label, marker=my_marker, capsize=5);

def plotTheoreticalLimit(nRobots,a,my_marker,axes):
  K_is_calculated_from_d_v = False
  maxVelocityLine = 12
  minDistanceLine = 11
  radius_s = 3
  maxvs = [max(data[n, a, :nSamples[a], maxVelocityLine]) for n in range(len(nRobots))]
  minds = [min(data[n, a, :nSamples[a], minDistanceLine]) for n in range(len(nRobots))]
  if K_is_calculated_from_d_v:
    holoSubStr = "nonholo" if "nonholo" in algorithms[a] else "holo"
    if os.path.isfile('bin/Ks'+holoSubStr+'.bin'):
      Ks = loadFile('bin/Ks'+holoSubStr+'.bin')
    else: 
      Ks = [Kmax(radius_s, minds[n]) for n in range(len(nRobots))]
      saveFile('bin/Ks'+holoSubStr+'.bin',Ks)
    limits = [limitF(Ks[n], radius_s, maxvs[n], minds[n]) for n in range(len(nRobots))]
    minminds = min(minds)
  else:
    limits = [limitF(5, radius_s, maxvs[n], minds[n]) for n in range(len(nRobots))]
  axes.plot(nRobots,limits,color='tab:orange',marker=my_marker)

def mainLoop(fileline):
  # ~ plt.figure(figsize=(7.5,4.5));
  plt.rcParams.update({'font.size': 15})
  plotMaxLimit = False

  for a in range(len(algorithms)):
    if fileline != 19:
      plt.errorbar(nRobots,dataMean[:,a,fileline], yerr=[m - n for m,n in zip(dataUpCi[:,a,fileline],dataMean[:,a,fileline])], label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5);
    else:
      if plotMaxLimit:
        fig, (ax1, ax2) = plt.subplots(2, 1)
        ax2.errorbar(nRobots,dataMean[:,a,fileline], yerr=[m - n for m,n in zip(dataUpCi[:,a,fileline],dataMean[:,a,fileline])],marker=algorithmsSymbol[a],capsize=5);
        ax2.set_title("Experiments")
        plotTheoreticalMeanLimits(nRobots,a,algorithmsSymbol[a],"Mean values",ax2)
        plotTheoreticalLimit(nRobots,a,algorithmsSymbol[a],ax1)
        ax1.set_title("Asymptotic")
        for ax in fig.get_axes():
          ax.label_outer()
      else:
        plotTheoreticalMeanLimits(nRobots,a,algorithmsSymbol[a],"Asymptotic")
        plt.errorbar(nRobots,dataMean[:,a,fileline], yerr=[m - n for m,n in zip(dataUpCi[:,a,fileline],dataMean[:,a,fileline])],label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5);
    if fileline != 19 or not (fileline == 19 and plotMaxLimit):
      plt.legend(loc=0);
    plt.xlabel("Number of robots");
    list_line_ylabel = [ 
      #           Label                                  # index
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
      "Mean distance (m)",                               # 13
      "St. dev. of distance (m)",                        # 14
      "Number of samples for distance",                  # 15
      "Mean velocity (m/s)",                             # 16
      "St. dev. of velocity (m/s)",                      # 17
      "Number of samples for velocity",                  # 18
      "Throughput (1/s)"                                 # 19
    ]
    # ~ print(list_line_ylabel[fileline])
    # ~ print(testTEqual(dataMean[:,0,fileline], dataVari[:,0,fileline], nSamples[0], dataMean[:,1,fileline], dataVari[:,1,fileline], nSamples[1]))
    plt.ylabel(list_line_ylabel[fileline])
    holoSubStr = "nonholo" if "nonholo" in algorithms[a] else "holo"
    if fileline != 19:
      plt.savefig("FigureRobots"+str(fileline)+holoSubStr+".pdf",bbox_inches="tight",pad_inches=0.01);
      plt.savefig("FigureRobots"+str(fileline)+holoSubStr+".png",bbox_inches="tight",pad_inches=0.01);
    else:
      plt.savefig("TRVF-throughput-th-exp-"+holoSubStr+".pdf",bbox_inches="tight",pad_inches=0.01);
      plt.savefig("TRVF-throughput-th-exp-"+holoSubStr+".png",bbox_inches="tight",pad_inches=0.01);
    plt.clf();

mainLoop(19)

