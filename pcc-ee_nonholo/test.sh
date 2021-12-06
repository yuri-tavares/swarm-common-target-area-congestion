#execute tests and interrupts if execution run over a specified time

if [ "$6" == "video" ]; then
  video="video"
fi

if [ "$video" != "video" ]; then
  GUI="-g"
fi

if [ "$6" == "gui" ]; then
  GUI=""
fi

export STAGEPATH="$PWD/"
./createScenario $1 $2 $3 $4 $5 $video
if [ $? -eq 0 ]; then
  stage $GUI $1.world 
fi
