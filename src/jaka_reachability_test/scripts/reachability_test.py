import geometry_msgs.msg
import moveit_commander
from tf.transformations import quaternion_from_euler
import numpy as np
import csv
import rospy
import argparse
import os


def main():
    parse = argparse.ArgumentParser(
        description='''
        Reachability test for jaka robotic arm. 
        This program will generate random points(x,y,z) with z specfied in --at-heights arg and random poses(roll,pitch,yaw) for each point. 
        Then it will try to plan a path for each pose. If a path is found, it will be counted as a reachable pose. 
        Then the program will statistics the number of reachable poses for each point, and output the result to a csv file.
        Author: Haojie Zhang (Huajuan)
        '''
    )
    parse.add_argument('--group-name', type=str, default='jaka_zu7', help='group name, default jaka_zu7')
    parse.add_argument('--at-heights', type=float, help='test jaka reachability at certain heights (z=h)', nargs='+', required=True)
    parse.add_argument('--point-num', type=int, help='number of random points to test, default 1000', default=1000)
    parse.add_argument('--pose-num-each-point', type=int, help='number of random poses to test for each point, default 100', default=100)
    parse.add_argument('--output-dir', type=str, help='csv data output directory', default='.')
    args = parse.parse_args()

    group_name = args.group_name
    output_dir = args.output_dir
    heights = args.at_heights
    point_num = args.point_num
    pose_num_each_point = args.pose_num_each_point

    rospy.init_node('reachability_test')
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_planning_time(0.05)

    headers = ["point_id", "x", "y", "z", "reachable_poses"]
    for h in heights:
        csv_filename = f"reachability_test_{group_name}_height_{str(h).replace('.','_')}.csv"
        file = open(os.path.join(output_dir, csv_filename), 'w')
        data = []
        csv_writer = csv.writer(file)
        for point_id in range(point_num):
            x = np.random.uniform(-1.0, 1.0)
            y = np.random.uniform(-1.0, 1.0)
            success_cnt = 0
            for _ in range(pose_num_each_point):
                roll = np.random.uniform(-np.pi, np.pi)
                pitch = np.random.uniform(-np.pi, np.pi)
                yaw = np.random.uniform(-np.pi, np.pi)
                q = quaternion_from_euler(roll, pitch, yaw)
                pose = geometry_msgs.msg.Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = h
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                plan = group.plan(pose)
                solved = plan and plan[0]
                if solved:
                    success_cnt += 1
            data.append([point_id, x, y, h, success_cnt])
            print(
                f"height: {h}, point_id: {point_id} x={x},y={y},z={h} done, found {success_cnt} reachable poses")
        csv_writer.writerow(headers)
        csv_writer.writerows(data)
        file.close()


if __name__ == '__main__':
    main()
