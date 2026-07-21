#!/usr/bin/env python3

import sys
import os
import pandas as pd

import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quaternion_to_yaw(q):
    """
    Convert quaternion to yaw angle
    """
    import math

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return math.atan2(siny_cosp, cosy_cosp)


def main():

    if len(sys.argv) != 2:
        print(
            "Usage:\n"
            "  python3 bag_to_csv.py <rosbag_folder>"
        )
        return


    bag_path = sys.argv[1]

    if not os.path.exists(bag_path):
        print("Bag not found:", bag_path)
        return


    print("Opening bag:")
    print(bag_path)


    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="sqlite3"
    )

    converter_options = rosbag2_py.ConverterOptions(
        "",
        ""
    )


    reader.open(
        storage_options,
        converter_options
    )


    topics = reader.get_all_topics_and_types()


    type_map = {}

    for topic in topics:

        type_map[topic.name] = get_message(
            topic.type
        )

        print(
            topic.name,
            " -> ",
            topic.type
        )


    cmd_vel_data = []
    odom_data = []
    tf_data = []


    print("Reading messages...")


    while reader.has_next():

        topic, data, timestamp = reader.read_next()


        if topic not in type_map:
            continue


        msg = deserialize_message(
            data,
            type_map[topic]
        )


        time_sec = timestamp * 1e-9


        # ==========================
        # cmd_vel
        # ==========================

        if topic == "/cmd_vel":

            cmd_vel_data.append({

                "time": time_sec,

                "linear_x":
                    msg.linear.x,

                "linear_y":
                    msg.linear.y,

                "angular_z":
                    msg.angular.z
            })


        # ==========================
        # odometry
        # ==========================

        elif topic == "/odom":

            pose = msg.pose.pose
            twist = msg.twist.twist


            odom_data.append({

                "time": time_sec,

                "x":
                    pose.position.x,

                "y":
                    pose.position.y,

                "yaw":
                    quaternion_to_yaw(
                        pose.orientation
                    ),


                "vx":
                    twist.linear.x,

                "vy":
                    twist.linear.y,

                "wz":
                    twist.angular.z
            })


        # ==========================
        # TF
        # ==========================

        elif topic == "/tf":

            for transform in msg.transforms:

                tf_data.append({

                    "time":
                        time_sec,

                    "parent":
                        transform.header.frame_id,

                    "child":
                        transform.child_frame_id,


                    "x":
                        transform.transform.translation.x,

                    "y":
                        transform.transform.translation.y,

                    "z":
                        transform.transform.translation.z
                })


    print("Writing CSV files...")


    if cmd_vel_data:

        df = pd.DataFrame(cmd_vel_data)

        df.to_csv(
            "cmd_vel.csv",
            index=False
        )

        print(
            "created cmd_vel.csv",
            len(df),
            "rows"
        )


    if odom_data:

        df = pd.DataFrame(odom_data)

        df.to_csv(
            "odom.csv",
            index=False
        )

        print(
            "created odom.csv",
            len(df),
            "rows"
        )


    if tf_data:

        df = pd.DataFrame(tf_data)

        df.to_csv(
            "tf.csv",
            index=False
        )

        print(
            "created tf.csv",
            len(df),
            "rows"
        )


    print("Done.")



if __name__ == "__main__":

    main()
