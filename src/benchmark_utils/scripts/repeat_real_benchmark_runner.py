#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
import sys
import time
from datetime import datetime

import rospy


class RepeatRealBenchmarkRunner:
    def __init__(self):
        rospy.init_node("repeat_real_benchmark_runner", anonymous=False)

        self.repeats = int(rospy.get_param("~repeats", 10))
        self.pause_between_runs = float(rospy.get_param("~pause_between_runs", 3.0))

        self.odom_topic = rospy.get_param("~odom_topic", "/vio/odometry_enu")
        self.position_cmd_topic = rospy.get_param("~position_cmd_topic", "/position_cmd")
        self.controller_name = rospy.get_param("~controller_name", "nmpc")
        self.tasks_file = rospy.get_param("~tasks_file", "")
        self.log_dir = rospy.get_param("~log_dir", os.path.expanduser("~/benchmark_results"))
        self.rate = rospy.get_param("~rate", 100)
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.run_analysis = rospy.get_param("~run_analysis", True)
        self.record_bag = rospy.get_param("~record_bag", False)
        self.bag_dir = rospy.get_param("~bag_dir", self.log_dir)
        self.vrpn_object = rospy.get_param("~vrpn_object", "droneyee08")
        self.auto_start_delay = float(rospy.get_param("~auto_start_delay", 2.0))
        self.batch_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.batch_root_dir = os.path.join(self.log_dir, self.batch_timestamp)
        self.batch_bag_root_dir = os.path.join(self.bag_dir, self.batch_timestamp)
        os.makedirs(self.batch_root_dir, exist_ok=True)
        os.makedirs(self.batch_bag_root_dir, exist_ok=True)

    def _build_command(self, run_index):
        run_root_dir = os.path.join(self.batch_root_dir, "repeat_runs", "run_%02d" % run_index)
        run_bag_dir = os.path.join(self.batch_bag_root_dir, "repeat_runs", "run_%02d" % run_index)
        os.makedirs(run_root_dir, exist_ok=True)
        os.makedirs(run_bag_dir, exist_ok=True)

        return [
            "roslaunch",
            "benchmark_utils",
            "real_benchmark.launch",
            "odom_topic:=%s" % self.odom_topic,
            "position_cmd_topic:=%s" % self.position_cmd_topic,
            "controller_name:=%s" % self.controller_name,
            "tasks_file:=%s" % self.tasks_file,
            "log_dir:=%s" % run_root_dir,
            "use_timestamp_subdir:=false",
            "rate:=%s" % self.rate,
            "frame_id:=%s" % self.frame_id,
            "run_analysis:=%s" % str(self.run_analysis).lower(),
            "record_bag:=%s" % str(self.record_bag).lower(),
            "bag_dir:=%s" % run_bag_dir,
            "vrpn_object:=%s" % self.vrpn_object,
            "auto_start:=true",
            "auto_start_delay:=%s" % self.auto_start_delay,
            "shutdown_on_finish:=true",
        ]

    def run(self):
        rospy.loginfo("=" * 70)
        rospy.loginfo("[RepeatBench] 开始批量仿真测试，共 %d 次", self.repeats)
        rospy.loginfo("[RepeatBench] 本批次目录: %s", self.batch_root_dir)
        rospy.loginfo("=" * 70)

        for idx in range(1, self.repeats + 1):
            if rospy.is_shutdown():
                break

            cmd = self._build_command(idx)
            rospy.loginfo("[RepeatBench] 第 %d/%d 次开始", idx, self.repeats)
            rospy.loginfo("[RepeatBench] 命令: %s", " ".join(cmd))

            ret = subprocess.call(cmd)
            if ret != 0:
                rospy.logerr("[RepeatBench] 第 %d 次失败，退出码: %d", idx, ret)
                sys.exit(ret)

            rospy.loginfo("[RepeatBench] 第 %d 次完成", idx)
            if idx < self.repeats:
                rospy.loginfo("[RepeatBench] 等待 %.1f s 后进入下一次", self.pause_between_runs)
                time.sleep(self.pause_between_runs)

        rospy.loginfo("[RepeatBench] 全部批量测试完成")


if __name__ == "__main__":
    try:
        RepeatRealBenchmarkRunner().run()
    except rospy.ROSInterruptException:
        pass
