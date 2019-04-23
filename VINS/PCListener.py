#!/usr/bin/env python

import rospy
import sensor_msgs.msg as s_msgs
import nav_msgs.msg as n_msgs
import geometry_msgs.msg as g_msgs
from sets import Set
from collections import OrderedDict
import csv

class PCListener:

    def __init__(self):
        """ The Constructor
        """
        self.output=[] #The list for storing all the points in the point cloud
        self.seq_tracker = OrderedDict() #For tracking the sequence number(i.e. which image the point cloud corresponds to)
        self.vins_odom=OrderedDict() #Record vins' estimate of pose
        self.global_odom = OrderedDict() #Record the final estimate of pose after fusing GPS with vins
        self.trans = OrderedDict() #Record the transformation between vins and global
        rospy.init_node('PCListener', anonymous=True)
        self.prevIDs = Set()

    def _pc_callback_full_pointcloud(self, data):
        """ Callback for point cloud messages

            :param data:
            :return:
        """
        for i in range(len(data.channels)):
            temp = [data.header.seq, data.header.stamp.secs, data.header.stamp.nsecs, data.channels[i].values[4],\
                data.channels[i].values[2], data.channels[i].values[3],\
                data.points[i].x,data.points[i].y, data.points[i].z]
            self.output.append(temp)

    # def _pc_callback(self, data):
    #     """ Callback for point cloud messages

    #         :param data:
    #         :return:
    #     """
    #     currIDs_dict = {}
    #     for i in range(len(data.channels)):
    #         currIDs_dict[int(data.channels[i].values[4])] = i
        
    #     currIDs = Set(currIDs_dict.keys())
    #     diff_set = currIDs.difference(self.prevIDs)

    #     for id in diff_set:
            
    #         index = currIDs_dict[id]
    #         temp = [data.header.seq, data.header.stamp.secs, data.header.stamp.nsecs, id,\
    #             data.channels[index].values[2], data.channels[index].values[3],\
    #             data.points[index].x,data.points[index].y, data.points[index].z]
            
    #         self.output.append(temp)

    #     self.prevIDs=currIDs
    
    def _pc_callback_track(self,data):
        self.seq_tracker[(data.header.stamp.secs, data.header.stamp.nsecs)] = data.header.seq
    
    def _ge_callback(self, data):
        self.global_odom[(data.header.stamp.secs, data.header.stamp.nsecs)] = \
            [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, \
             data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]

    
    def _ve_callback(self,data):
        self.vins_odom[(data.header.stamp.secs, data.header.stamp.nsecs)] = \
            [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
    def _trans_callback(self,data):
        self.trans[(data.header.stamp.secs, data.header.stamp.nsecs)] = \
            [data.transform.translation.x,data.transform.translation.y,data.transform.translation.z,\
             data.transform.rotation.w, data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z]

    def listener(self):
        """ Subscribe to Vins-Fusion Point Cloud Topics.

            :return:
        """
        rospy.Subscriber('/vins_estimator/keyframe_point', s_msgs.PointCloud, self._pc_callback_full_pointcloud,\
            queue_size=1, buff_size=2 ** 20)
        rospy.Subscriber('/vins_estimator/point_cloud', s_msgs.PointCloud, self._pc_callback_track,\
            queue_size=1, buff_size=2 ** 20)
        rospy.Subscriber('/globalEstimator/global_odometry', n_msgs.Odometry, self._ge_callback,\
            queue_size=1, buff_size=2 ** 20)
        rospy.Subscriber('/vins_estimator/odometry', n_msgs.Odometry, self._ve_callback,\
            queue_size=1, buff_size=2 ** 20)
        rospy.Subscriber('/globalEstimator/WGPS_T_WVIO', g_msgs.TransformStamped, self._trans_callback,\
            queue_size=1, buff_size=2 ** 20)
        
        rospy.spin()
    
    def exporter(self, csv_name='VF_pointcloud_expanded.csv'):
        """ Export the data to csv file

            :return:
        """
        with open('GPS_VIO_WGPS_T_WVIO.csv', 'w') as csvfile:
            posewriter = csv.writer(csvfile, delimiter=',')
            posewriter.writerow(['sequence','timestamp_sec','timestamp_nsec','global_position_x',\
                    'global_position_y', 'global_position_z','global_orientation_w','global_orientation_x',\
                        'global_orientation_y','global_orientation_z','vio_position_x', 'vio_position_y',\
                        'vio_position_z','trans_x','trans_y','trans_z', 'rot_q_w', 'rot_q_x','rot_q_y', 'rot_q_z'])
            for k1,v1 in self.global_odom.iteritems():
                try:
                    row = [self.seq_tracker[(k1[0],k1[1])], k1[0], k1[1],v1[0],v1[1],v1[2],v1[3],v1[4],v1[5],v1[6],\
                            self.vins_odom[k1][0],self.vins_odom[k1][1],self.vins_odom[k1][2],\
                            self.trans[k1][0],self.trans[k1][1],self.trans[k1][2],self.trans[k1][3],\
                                self.trans[k1][4],self.trans[k1][5],self.trans[k1][6]]
                except KeyError:
                    continue
                posewriter.writerow(row)

        with open(csv_name, 'w') as csvfile:
            pcwriter = csv.writer(csvfile, delimiter=',')
            pcwriter.writerow(['sequence', 'timestamp_sec','timestamp_nsec',\
                'uniqueID','pixels_u','pixels_v','x','y','z','global_position_x',\
                    'global_position_y', 'global_position_z','global_orientation_w','global_orientation_x',\
                        'global_orientation_y','global_orientation_z', 'vio_pose_x', 'vio_pose_y',\
                        'vio_pose_z','trans_x','trans_y','trans_z', 'rot_q_w', 'rot_q_x','rot_q_y', 'rot_q_z'])
            for row in self.output:
                row[0]=self.seq_tracker[(row[1],row[2])]
                try:
                    for coordinate in self.global_odom[(row[1],row[2])]:
                        row.append(coordinate)
                    for coordinate in self.vins_odom[(row[1],row[2])]:
                        row.append(coordinate)
                    for entry in self.trans[(row[1],row[2])]:
                        row.append(entry)

                except KeyError:
                    continue
                
                pcwriter.writerow(row)
    

if __name__ == '__main__':
    pcl = PCListener()
    pcl.listener()
    pcl.exporter()