#!/bin/bash
# The optional argument of this script is a file name with overrides for variables below.
# In this script, if the vector variable samples is defined, it is used for computed the increment for each value of the variable given. Otherwise, it is used the vector increment[i] for each index of variable i.

### Default values ###
# maximum number of retries if the log is empty
RETRYTIMES=1;
# Variables names for test.sh. Angle is measured in degrees
defaultVarNames=(s v K  n  d D holo)
# default values for each variable
defaultValues=(  3 1 4  100 3 13  0)
# number of repetitions for each variable value
nTests=40
# variables' name to do more experiments
varsExp=( d  )
# minimum and maximum values for each variable to do more experiments
minExp=( 1.0 )     
maxExp=( 3.0 )  
#increment of above values, since the value minExp until is greater than maxExp 
increment=( 0.5 )   
# Name of the folder where the temporary configuration ini file named in variable configFile will be created
# and where the directories containg the logs will be created.
folderConf=.

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

# The name of the log for the experiments performed by this script is randomly generated until a not used is found.
experimentsLog=$(generateRandomFile $folderConf experimentsLog .txt)
scenarioFileName=$(generateRandomPrefixFile . automatic .world)



# Creates a config file from arguments.
# Argument 1: name of variable.
# Argument 2: value of this variable.
# Argument 3: name of configuration file.
# Argument 4: folder name for save the log file.
# Argument 5: log's filename.
makeConfigFile(){
  if [ $# -ne 5 ]; then
    echo "Wrong number of parameter for makeConfigFile." $# "are given."
    echo "Parameters were: " $@
    return 1
  fi
  local varName=$1
  local varValue=$2
  local configName=$3
  local folderLog=$4
  local logname=$5
  echo "log="$logname > $configName
  echo "folder="$folderLog >> $configName
  size=${#defaultVarNames[@]}
  for (( ii=0; ii<$size; ii++ )); do 
    if [ ${defaultVarNames[$ii]} != $varName ]; then
      echo ${defaultVarNames[$ii]}"="${defaultValues[$ii]} >> $configName 
    else
      echo $varName"="$varValue >> $configName
    fi
  done
  echo "scenario="$scenarioFileName >> $configName
}



# Peform experiments from given paramenters.
# Argument 1: number of experiments for each variable value.
# Argument 2: name of the variable being experimented.
# Argument 3: minimum value of the variable.
# Argument 4: maximum value of the variable.
# Argument 5: increment in variable.
# 
doExperiments(){
  if [ $# -ne 5 ]; then
    echo "Wrong number of parameter for doExperiments. " $# "are given."
    echo "Parameters were: " $@
    return 1
  fi
  local nTests=$1
  local varName=$2
  local minValue=$3
  local maxValue=$4
  local varInc=$5
  varValue=$minValue

  echo "Log for anaysis.sh $@" &>> $experimentsLog
  # The configuration file of test.sh is random too.
  configFile="configExperiment$RANDOM.ini"
  while [ -f "$folderConf/$configFile" ]; do configFile="configExperiment$RANDOM.ini"; done
  while [ `echo "$varValue $maxValue" | awk '{printf "%i", $1 <= $2}'` != 0  ];
  do
    logsFolder=$folderConf/$varName\_$varValue
    local i
    for (( i=0; i<$nTests; i++ )); 
    do
      if [ ! -e $logsFolder/log\_$i ]; then
        logsName=$(generateRandomFile . log)
        makeConfigFile $varName $varValue $folderConf/$configFile "$logsFolder" $logsName
        
        echo "******* test for "$varName" = "$varValue"  ********" &>> $experimentsLog
        if [ ! -d $logsFolder ]; then
          mkdir $logsFolder;
        fi
        bash test.sh $folderConf/$configFile  &>> $experimentsLog
        
        # check if another script saved on this current log file number while this experiment was running
        i2=$i
        while [ -f $logsFolder/log\_$i2 ]; do 
          i2=$((i2+1)); 
        done
        
        # it only saves the log if it is still needed
        if [ $i2 -le $((nTests-1)) ]; then
          mv $logsFolder/$logsName $logsFolder/log\_$i2
          if [ -f $logsFolder/$logsName\_robot0_v ]; then
            pushd $logsFolder
            rename 's/'`basename $logsName`'_robot/log_'$i2'_robot/' $logsName\_robot*\_{d,v};
            popd
          fi
          #after it is created, if the file was  empty, it runs again at most RETRYTIMES
          try=0;
          while [ $try -lt $RETRYTIMES ] &&  [ ! -s $logsFolder/log\_$i2  ]  ; do
            echo "*** Running again. Log file named "$logsFolder/$logsName" was empty. ***" &>> $experimentsLog
            bash test.sh $folderConf/$configFile &>> $experimentsLog
            mv $logsFolder/$logsName $logsFolder/log\_$i2 
            if [ -f $logsFolder/$logsName\_robot0_v ]; then
              pushd $logsFolder
              rename 's/'`basename $logsName`'_robot/log_'$i2'_robot/' $logsName\_robot*\_{d,v};
              popd
            fi
            try=$((try+1));
          done;
        fi;
      fi;
    done
    varValue=`echo "$varValue $varInc" | awk '{printf "%.3f", $1 + $2}'`
  done
}

mkdir -p $folderConf
# initalise log for this script
cat /dev/null > $experimentsLog
sizeVars=${#varsExp[@]}
for (( j=0; j<$sizeVars; j++ )); do
  if [ ! -z $samples ]; then
    varInc=`echo "${maxExp[$i]} ${minExp[$i]} ${samples[$i]}" | awk '{printf "%.3f", ($1-$2)/($3 - 1) }'`
  else
    varInc=${increment[$j]}
  fi
  doExperiments $nTests ${varsExp[$j]} ${minExp[$j]} ${maxExp[$j]} $varInc 
done

