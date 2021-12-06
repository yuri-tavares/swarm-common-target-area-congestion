'''
Check the number of data in velocity and distance files obtained by Stage simulator given a directory and the prefix of the subdirectory containing the files.
'''

import sys
import numpy as np
import glob
import linecache

nRobotsStart = 20
nRobotsEnd   = 300
nRobotsInc   = 20
numTests     = 40

if len(sys.argv) < 3:
  print("Usage:")
  print("python3 " + sys.argv[0] + " [container directory] [subdirectory prefix]")
  exit()

container_dir = sys.argv[1]
subdir_prefix = sys.argv[2]

for n in range(nRobotsStart,nRobotsEnd+1,nRobotsInc):
  for l in range(numTests):
    logFile = container_dir+'/'+subdir_prefix+str(n)+'/log_'+str(l)
    regExForGlobPrefix = container_dir+'/'+subdir_prefix+str(n)+'/log_'+str(l)+'_robot*'
    
    # get variables from log
    minDistance     = float(linecache.getline(logFile,12))
    maxVelocity     = float(linecache.getline(logFile,13))
    meanDistance    = float(linecache.getline(logFile,14))
    sqrtvarDistance = float(linecache.getline(logFile,15))
    nDistance       = int(linecache.getline(logFile,16))
    meanVelocity    = float(linecache.getline(logFile,17))
    sqrtvarVelocity = float(linecache.getline(logFile,18))
    nVelocity       = int(linecache.getline(logFile,19))
    
    Compare1 = (minDistance, maxVelocity, meanDistance, sqrtvarDistance, nDistance, meanVelocity, sqrtvarVelocity, nVelocity)
    
    #get velocities from every robot
    regExForGlob = regExForGlobPrefix+'_v'
    A = [float(a) for f in glob.glob(regExForGlob) for a in open(f).read().splitlines()] 
    meanVelRobots, stdVelRobots, maxVelRobots, lenVelRobots = np.mean(A), np.sqrt(np.var(A)), max(A), len(A)
    del A
    
    #get distances from every robot
    regExForGlob = regExForGlobPrefix+'_d'
    A = [float(a) for f in glob.glob(regExForGlob) for a in open(f).read().splitlines()] 
    meanDistRobots, stdDistRobots, minDistRobots, lenDistRobots = np.mean(A), np.sqrt(np.var(A)), min(A), len(A)
    del A
    
    Compare2 = (minDistRobots, maxVelRobots, meanDistRobots, stdDistRobots, lenDistRobots, meanVelRobots, stdVelRobots, lenVelRobots)
    
    if max([abs(c1 - c2) for c1, c2 in zip(Compare1, Compare2)]) > 1e-05: 
      print(logFile)
      print(Compare1)
      print(Compare2)
