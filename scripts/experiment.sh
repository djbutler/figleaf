# $1 is the user id
# $2 is the name of the flight
# $3 is the name of the filter (clean, blur, ...)
# $4 is the name of the task (func, adv)

DATA_DIR=/home/$USER/experiment_data

if [ ! -e $DATA_DIR ]
then
  mkdir $DATA_DIR
fi

if [ -e $DATA_DIR/user$1_$2_$3_$4.bag ]
then
  echo "/home/$USER/experiment_data/user$1_$2_$3_$4.bag already exists! Stopping launch"
else
  ./reset_robot.sh
  case "$3" in
    clean)
      roslaunch figleaf_2d experiment.launch user:=$1 data_dir:=$DATA_DIR flight:=$2 filter:=$3 clean:=True task:=$4
      ;;
    blur)
      roslaunch figleaf_2d experiment.launch user:=$1 data_dir:=$DATA_DIR flight:=$2 filter:=$3 blur:=True task:=$4
      ;;
    mid)
      roslaunch figleaf_2d experiment.launch user:=$1 data_dir:=$DATA_DIR flight:=$2 filter:=$3 mid:=True task:=$4
      ;;
    box)
      roslaunch figleaf_2d experiment.launch user:=$1 data_dir:=$DATA_DIR flight:=$2 filter:=$3 box:=True task:=$4
      ;;
  esac
fi
