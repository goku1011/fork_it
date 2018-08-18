import rospy
import numpy as np
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import xlwt
import matplotlib.pylab as pl
import matplotlib.gridspec as gridspec

rospy.init_node("camera_info_publisher", anonymous=True)

ndt_Pub = rospy.Publisher("/ndt", PoseStamped, queue_size=10)
gnss_Pub = rospy.Publisher("/gnss", PoseStamped, queue_size=10)
integrated_Pub = rospy.Publisher("/integrated", PoseStamped, queue_size=10)

def gnss_callback(data):
    gnss_x.append(data.pose.position.x)
    gnss_y.append(data.pose.position.y)

def ndt_callback(data):
    ndt_x.append(data.pose.position.x)
    ndt_y.append(data.pose.position.y)

def integrated_callback(data):
    integrated_x.append(data.pose.position.x)
    integrated_y.append(data.pose.position.y)


if __name__ == "__main__":

    gnss_x = []
    gnss_y = []
    ndt_x = []
    ndt_y = []
    integrated_x = []
    integrated_y = []

    #gnss_subs = subscriber("/gnss_pose",PoseStamped)
    #ndt_subs = subscriber("/ndt_pose",PoseStamped)
    #integrated_subs = subscriber("/integrated_pose",PoseStamped)

    rospy.Subscriber("/gnss_pose", PoseStamped, gnss_callback)
    rospy.Subscriber("/ndt_pose", PoseStamped, ndt_callback)
    rospy.Subscriber("/integrated_pose", PoseStamped, integrated_callback)
    rospy.spin()

    if(rospy.core.is_shutdown()):

        gs = gridspec.GridSpec(1,1)
        f = pl.figure(1)
        line_p, = pl.plot(gnss_x,gnss_y,'r.', label='gnss')
        line_d, = pl.plot(ndt_x,ndt_y,'b.', label='ndt')
        line_n, = pl.plot(integrated_x,integrated_y,'k.', label='integrated')
        pl.legend(handles=[line_p, line_d, line_n])
        pl.grid()

        gs = gridspec.GridSpec(1,1)
        f = pl.figure(2)
        line_n, = pl.plot(integrated_x,integrated_y,'k--', label='integrated')
        pl.legend(handles=[line_n])
        pl.grid()

        gs = gridspec.GridSpec(1,1)
        f = pl.figure(3)
        line_d, = pl.plot(ndt_x,ndt_y,'b--', label='ndt')
        pl.legend(handles=[line_d])
        pl.grid()

        gs = gridspec.GridSpec(1,1)
        f = pl.figure(4)
        line_p, = pl.plot(gnss_x,gnss_y,'r--', label='gnss')
        pl.legend(handles=[line_p])
        pl.grid()

        gs = gridspec.GridSpec(1,1)
        f = pl.figure(5)
        line_p, = pl.plot(gnss_x,gnss_y,'r.', label='gnss')
        line_d, = pl.plot(ndt_x,ndt_y,'b.', label='ndt')
        line_n, = pl.plot(integrated_x,integrated_y,'k--', label='integrated')
        pl.legend(handles=[line_p, line_d, line_n])
        pl.grid()

        pl.show()
