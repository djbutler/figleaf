DATA_DIR=/home/$USER/experiment_data

if [ ! -e $DATA_DIR ]
then
  mkdir $DATA_DIR
fi

if [ -e $DATA_DIR/$1_$2.bag ]
then
  echo "/home/$USER/experiment_data/$1_$2.bag already exists! Stopping launch"
else
  roslaunch figleaf_2d experiment.launch user:=$1 filter:=$2 data_dir:=$DATA_DIR
fi
