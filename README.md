## Anonymous Repo for Stealthy Terrain-Aware Multi-Agent Active Search

This repo has been anonymized and hence may not be runnable.

In order to run the repo run the dependencies are ROS melodic and python2.7 with numpy, matplotlib and PIL.

To recreate the experiments or just run the algorithm you should use docker:
- Install docker following this link: https://docs.docker.com/engine/install/
- Recommended OS is Ubuntu 18.04
- Then run the following command to download the necessary image and start the container
```
bash start_docker.sh
```
- Inside the docker container:
```
cd /home/user/src
bash init.sh
```
- Ignore any pip install errors that occur if they do, then:
```
cd zone_recon
source devel/setup.bash
USER=user ROS_LOG_DIR=/home/user/src/logging bash test_mysim.sh 100 star 5 4
```
- The syntax for the above command is `test_mysim.sh <runtime budget in seconds> <algorithm: star, rsi, guts, coverage, random> <number of targets> <number of agents: 1 2 4 or 8>`
More detailed instructions to follow to recreate experiments.
- Feel free to try any of combinations. The runs can be seen in the logging folder such that every robot has logs (replicating the real life system).
- The results for the paper were generated using the `experiments{_multi,_simplesim}.sh` scripts and they take several days for realistic simulations and several hours for the simplified simulator.
- The main code for STAR, GUTS, RSI, COVERAGE and RANDOM policies tested for adversarial multi agent active search in this paper can be found written in src/zone_recon/src/waypoint_planner/src/waypoint_planner/active_search.py.
