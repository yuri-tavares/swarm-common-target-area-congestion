#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from math import tan, sin, asin, cos, pi, sqrt, floor, ceil, hypot, log, fabs, fmod
import glob
from array import array
import os.path

def readLogs(path):
  timelimit = 10000000000
  #Save all data from individual robots logs
  robotFiles = glob.glob(path+"/"+"/robot*")
  robotData = []
  for f in robotFiles:
    robotFile = open(f)
    dataFileStr = robotFile.readlines();
    data = []
    for i in range(4):
      data.append(int(dataFileStr[i])) 
    robotData.append(data)
  # Sort robot data by target arrival time
  robotDataSorted = sorted(robotData, key= lambda x: x[3]) 
  # Remove data for robots that reached the time limit
  while robotDataSorted[-1][3] > timelimit + robotDataSorted[0][3]: robotDataSorted.pop() 
  #count robots with same spent time
  uniqueTime = sorted(set([i[3] for i in robotDataSorted]))
  dicttime = {t:0 for t in uniqueTime}
  for i in robotDataSorted: dicttime[i[3]]+=1
  increasingNumRobotsPerTime = [dicttime[uniqueTime[0]]]
  for t in range(1,len(uniqueTime)):
    increasingNumRobotsPerTime.append(increasingNumRobotsPerTime[t-1] + dicttime[uniqueTime[t]])
  return (increasingNumRobotsPerTime, uniqueTime)

def fparallel(T,s,d,v):
  I = intfloor(s/d) if fabs(s - floor(s/d)*d) <= fabs(s - ceil(s/d)*d) else intceil(s/d)
  dI = s - sqrt(s**2 - (s-I*d)**2)
  n = 0
  for i in range(0, floor(2*s/d)+1):
    di = s - sqrt(s**2 - (s-i*d)**2)
    ni = floor((v*T -di + dI)/d) + 1 if T >= (di - dI)/v else 0
    n = n + ni
  return (1./T)*(n-1)

def saveFile(outfilename, out):
  output_file = open(outfilename, 'wb')
  float_array = array('d', out)
  float_array.tofile(output_file)
  output_file.close()

def loadFile(filename):
  input_file = open(filename, 'rb')
  float_array = array('d')
  float_array.fromstring(input_file.read())
  return float_array

'''
Plot simulation and theoretical values.
'''
def main():
  # ~ plt.rc('text', usetex=True)
  plt.rcParams.update({'font.size': 15})
  
  vs = ["0.1","1.0"]
  for v in vs:
    s = [0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95]
    nExp = len(s)
    d = [1]*nExp
    prefixPath = "v"+v+"/time10500/s_"
    upperDirectories = ["../hexagonal tiling","../parallel lanes"]
    paths = [prefixPath+str(sv)+"/" for sv in s]
    algorithmsTextLabels = r'$u$'
    outputLabels = "Throughput (1/s)"
    Color3 = 'mediumpurple'
    Color4 = 'brown'
    
    us=loadFile("../hexagonal tiling/bin/us_zoom_04_1_10000.bin")
    if v == "1.0":
      yhex = loadFile("../hexagonal tiling/bin/yhex_zoom_04_1_10000.bin") 
      ypa = loadFile("../parallel lanes/bin/ypa_zoom_04_1_10000.bin")
      thetaS = {0.4:0.0, 0.45:0.441311480534, 0.5:0.5235987755982988, 0.55:0.5235987755982988, 0.6:0.5235987755982988, 0.65:0.5235987755982988, 0.7:0.5235987755982988, 0.75:0.441311480534, 0.8:0.226421092151, 0.85:0.226421092151, 0.9:0.0, 0.95:0.0}
    else:
      yhex = loadFile("../hexagonal tiling/bin/yhex_zoom_04_1_10000v0.1.bin") 
      ypa = loadFile("../parallel lanes/bin/ypa_zoom_04_1_10000v0.1.bin")
      thetaS = {0.4: 1.0471975512, 0.45: 0.441311480534, 0.5: 0.5235987755982988, 0.55: 0.5235987755982988, 0.6: 0.5235987755982988, 0.65: 0.5235987755982988, 0.7: 0.5235987755982988, 0.75: 0.441311480534,0.8: 0.156188623752, 0.85: 0.427684285173, 0.9: 1.0471975512, 0.95: 1.0471975512}
    
    plt.plot(us,yhex,label=r'$f_h(10000,u)$')
    plt.plot(us,ypa,label=r'$f_p(10000,u)$')
    
    for upp in upperDirectories:
      print("---------",upp,v,"---------")
      if v == "1.0" and os.path.isfile(upp+"/bin/Tpsforv1.0.bin"):
        Tps = loadFile(upp+"/bin/Tpsforv1.0.bin");
      else:
        Tps = []
        for a in range(len(paths)):
          (increasingNumRobotsPerTime, uniqueTime) = readLogs(upp+"/"+paths[a])
          V = len(uniqueTime)
          Tv = (uniqueTime[V-1] - uniqueTime[0])/1e6
          Nv = increasingNumRobotsPerTime[V-1]
          Tp = (Nv-1)/Tv
          Tps.append(Tp)
        if v == "1.0":
          saveFile(upp+"/bin/Tpsforv1.0.bin",Tps)
      if upp == '../hexagonal tiling':
        suffix = ' hex.'
        markerHere = 'o'
        colorHere=Color3
      else:
        suffix = ' par.'
        markerHere = 'x'
        colorHere=Color4
      plt.plot(s,Tps,label='Simulation'+suffix,marker=markerHere,linestyle="None",color=colorHere)
    
    plt.legend(loc=0,prop={'size': 13});
    plt.xlabel(algorithmsTextLabels);
    plt.ylabel(outputLabels);
    plt.savefig("SimHexPar"+v+".pdf",bbox_inches="tight",pad_inches=0.01);
    # ~ plt.show()
    plt.clf()

'''
Plot only theoretical values.
'''
def main2():
  # ~ plt.rc('text', usetex=True)
  plt.rcParams.update({'font.size': 15})
  
  vs = ["0.1","1.0"]
  for v in vs:
    s = [0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95]
    nExp = len(s)
    d = [1]*nExp
    prefixPath = "v"+v+"/time10500/s_"
    upperDirectories = ["../hexagonal tiling","../parallel lanes"]
    paths = [prefixPath+str(sv)+"/" for sv in s]
    algorithmsTextLabels = r'$u$'
    outputLabels = "Throughput (1/s)"
    Color3 = 'mediumpurple'
    Color4 = 'brown'
    
    us=loadFile("../hexagonal tiling/bin/us_zoom_04_1_10000.bin")
    if v == "1.0":
      yhex = loadFile("../hexagonal tiling/bin/yhex_zoom_04_1_10000.bin") 
      ypa = loadFile("../parallel lanes/bin/ypa_zoom_04_1_10000.bin")
      thetaS = {0.4:0.0, 0.45:0.441311480534, 0.5:0.5235987755982988, 0.55:0.5235987755982988, 0.6:0.5235987755982988, 0.65:0.5235987755982988, 0.7:0.5235987755982988, 0.75:0.441311480534, 0.8:0.226421092151, 0.85:0.226421092151, 0.9:0.0, 0.95:0.0}
    else:
      yhex = loadFile("../hexagonal tiling/bin/yhex_zoom_04_1_10000v0.1.bin") 
      ypa = loadFile("../parallel lanes/bin/ypa_zoom_04_1_10000v0.1.bin")
      thetaS = {0.4: 1.0471975512, 0.45: 0.441311480534, 0.5: 0.5235987755982988, 0.55: 0.5235987755982988, 0.6: 0.5235987755982988, 0.65: 0.5235987755982988, 0.7: 0.5235987755982988, 0.75: 0.441311480534,0.8: 0.156188623752, 0.85: 0.427684285173, 0.9: 1.0471975512, 0.95: 1.0471975512}
    
    plt.plot(us,yhex,label=r'$f_h(10000,u)$')
    plt.plot(us,ypa,label=r'$f_p(10000,u)$')
    
    plt.legend(loc=0,prop={'size': 13});
    plt.xlabel(algorithmsTextLabels);
    plt.ylabel(outputLabels);
    plt.savefig("ThOnlyHexPar"+v+".pdf",bbox_inches="tight",pad_inches=0.01);
    # ~ plt.show()
    plt.clf()


main2();
