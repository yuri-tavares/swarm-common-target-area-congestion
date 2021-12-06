#!/bin/bash
# The optional argument of this script is a file name with overrides for variables below.

MAXTRIES=1;
NUMTESTS=40;
nRobotsStart=20;
nRobotsEnd=140;
radius=3

# The configuration file passed in first argument overrides the above values
if [ ! -z $1  ]; then
  . $1
fi

# Creates a randomly numbered file from arguments.
# Argument 1: directory for checking the filename existence.
# Argument 2: filename prefix.
# Argument 3: filename suffix.
# Return: the generated random file name is "echoed".
generateRandomFile(){
  local res="$1/$2$RANDOM$3"
  while [ -f "$res" ]; do res="$1/$2$RANDOM$3"; done
  echo $res
}

# Creates a randomly numbered prefix for a file from arguments.
# Argument 1: directory for checking the filename existence.
# Argument 2: filename prefix.
# Argument 3: filename suffix.
# Return: the generated random prefix of the filename is "echoed".
generateRandomPrefixFile(){
  local res="$1/$2$RANDOM"
  while [ -f "$1/$2$RANDOM$3" ]; do res="$1/$2$RANDOM"; done
  echo $res
}

analysisLogFile=$(generateRandomFile . logTest)
auto=$(generateRandomPrefixFile . automatic .world)

cat /dev/null > $analysisLogFile

for ((nRobos=$nRobotsStart;nRobos<=$nRobotsEnd;nRobos+=20))
do 
  echo "*** Experimenting the time increasing in relation to the number of robots: $nRobos robots. ****"
  for j in `seq 0 $((NUMTESTS-1))`; 
  do
    if [ ! -e nRobos$nRobos/log\_$j ]; then
      logfile=$(generateRandomFile . log)
      echo "******* test.sh $auto $nRobos $logfile $radius ********" &>> $analysisLogFile
      ./test.sh $auto $nRobos $logfile $radius &>> $analysisLogFile
      if [ ! -d nRobos$nRobos ]; then
        mkdir nRobos$nRobos;
      fi
      # check if another script saved on this current log file number while this experiment was running
      i=$j
      while [ -f nRobos$nRobos/log\_$i ]; do 
        i=$((i+1)); 
      done
      
      # it only saves the log if it is still needed
      if [ $i -le $((NUMTESTS-1)) ]; then
        mv $logfile nRobos$nRobos/log\_$i;
        if [ -f robot0_v ]; then
          rename 's/^/log_'$i'_/' robot*\_{d,v};
          mv log\_$i\_robot*\_{d,v} nRobos$nRobos/;
        fi

        #After it is created, if the log file is empty, the experiment is done again at most MAXTRIES
        try=0;
        while [ $try -lt $MAXTRIES ] && ( [ ! -s nRobos$nRobos/log\_$i  ] ) ; do
          echo "*** Running again. Log file was empty. ***" &>> $analysisLogFile
          ./test.sh $auto $nRobos $logfile $radius &>> $analysisLogFile
          mv log nRobos$nRobos/log\_$i;
          if [ -f robot0_v ]; then
            rename 's/^/log_'$i'_/' robot*\_{d,v};
            mv log\_$i\_robot*\_{d,v} nRobos$nRobos/;
          fi
          try=$((try+1));
        done;
      fi;
    fi;
  done
done; 

