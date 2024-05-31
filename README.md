# Classical-MPC

The code is developed to implement the classical MPC, which can control the agent to reach the specified goal while avoiding the static obstacles in MuJoCo environment of "PointNMaze-v0".


## Installation:

0. Create a conda environment:
```
conda create --name myenv python=3.8.18
conda activate myenv
```

1. Install subfolder dependencies:
```
pip install -r requirements.txt
```

## Run
Run the simulation:

```
python test_mpc2.py
```

