# Jaka Robot Reachability Test
This package is used to test the reachability of Jaka Robot. Author: Haojie Zhang (Huajuan)

The `reachability_test` program generate random points(x,y,z) with z specfied in --at-heights arg and random poses(roll,pitch,yaw) for each point. Then it will try to plan a path for each pose. If a path is found, it will be counted as a reachable pose. Then the program will statistics the number of reachable poses for each point, and output the result to a csv file.

## args
```
usage: reachability_test.py [-h] [--group-name GROUP_NAME] --at-heights AT_HEIGHTS [AT_HEIGHTS ...] [--point-num POINT_NUM]
                            [--pose-num-each-point POSE_NUM_EACH_POINT] [--output-dir OUTPUT_DIR]

Reachability test for jaka robotic arm. This program will generate random points(x,y,z) with z specfied in --at-heights arg and random poses(roll,pitch,yaw)
for each point. Then it will try to plan a path for each pose. If a path is found, it will be counted as a reachable pose. Then the program will statistics
the number of reachable poses for each point, and output the result to a csv file. Author: Haojie Zhang (Huajuan)

optional arguments:
  -h, --help            show this help message and exit
  --group-name GROUP_NAME
                        group name, default jaka_zu7
  --at-heights AT_HEIGHTS [AT_HEIGHTS ...]
                        test jaka reachability at certain heights (z=h)
  --point-num POINT_NUM
                        number of random points to test, default 1000
  --pose-num-each-point POSE_NUM_EACH_POINT
                        number of random poses to test for each point, default 100
  --output-dir OUTPUT_DIR
                        csv data output director
```

## example
First, launch the robot and moveit server and rviz:
```bash
roslaunch jaka_zu7_moveit_config demo.launch
```

Then, run the reachability test program:
```bash
rosrun jaka_reachability_test reachability_test.py --group-name jaka_zu7 --at-heights 0.3 0.7 --point-num 3 --pose-num-each-point 10 --output-dir ~
```
The above command will generate 3 random points at height 0.3 and 0.7, and for each point, it will generate 10 random poses and test if the robot can reach the pose. The result will be saved to a csv file in the home directory.

## output
The output csv file will be stored in the output directory specified by `--output-dir` arg. The file name is `reachability_test_${group_name}_height_${height}.csv`, with dot in `${height}` replaced by underscore. For example, the command in the above example section will generate two csv files: `reachability_test_jaka_zu7_height_0_3.csv` and `reachability_test_jaka_zu7_height_0_7.csv`. 

These csv files have the same format, with the following columns:
- `point_id`: the id of the point
- `x`: x coordinate of the point
- `y`: y coordinate of the point
- `z`: z coordinate of the point, specified by `--at-heights` arg
- `reachable_pose_num`: number of reachable poses for the point

The following is an example of the csv file:
```
point_id,x,y,z,reachable_poses
0,-0.02792647557033545,0.8814652191717667,0.3,0
1,0.16869787268558833,-0.07867391939300572,0.3,9
2,0.19189210435450366,0.3669074793826308,0.3,10
```
