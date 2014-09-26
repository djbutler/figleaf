# $1 is the user id
# $2 is the name of the filter (clean, blur, ...)

DATA_DIR=/home/$USER/experiment_data

if [ ! -e $DATA_DIR ]
then
  mkdir $DATA_DIR
fi

if [ -e $DATA_DIR/$1_$2.bag ]
then
  echo "/home/$USER/experiment_data/$1_$2.bag already exists! Stopping launch"
else
  ./reset_robot.sh
  case "$2" in
    clean)
      roslaunch figleaf_2d experiment.launch user:=$1 data_dir:=$DATA_DIR filter:=$2 clean:=True
      ;;
    blur)
      roslaunch figleaf_2d experiment.launch user:=$1 data_dir:=$DATA_DIR filter:=$2 blur:=True
      ;;
    mid)
      roslaunch figleaf_2d experiment.launch user:=$1 data_dir:=$DATA_DIR filter:=$2 mid:=True
      ;;
    box)
      roslaunch figleaf_2d experiment.launch user:=$1 data_dir:=$DATA_DIR filter:=$2 box:=True
      ;;
  esac
fi
