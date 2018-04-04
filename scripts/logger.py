#!/usr/bin/env python
#import pdb
import rospy
import copy, os
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Joy
from conf import Tasks
from datetime import datetime
    
f = None # file
arr = [0,0,0,0] # [m0, m1, m2, m3]
pub = rospy.Publisher('task', Int8, queue_size=10)
endButtonIndex = 0

def joy_callback(data):
    # check finish
    print 'jc'
    if (data.buttons[endButtonIndex] == 1):
        print 'end logging'
        pub.publish(Int8(Tasks.ODOMRUN_LOGGING_END))
        if f:
            f.close()
        else:
            print 'have not logging...'
        arr = [0,0,0,0]
    
def task_callback(data):    
    r = data.data
    global f
    if r == Tasks.ODOMRUN_LOGGING_BEGIN:
        print 'start logging'
        timestr = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        #f = open('odom.txt', 'a') # xxx rename...
        f = open(os.path.expanduser(str('~/catkin_ws/src/ar_main/scripts/'+timestr+'.txt')), 'a')
        print f
        
def value_callback(data):
    r = data.data
    for i in range(0,len(r)):
        arr[i] = copy.copy(r[i])
        
    #   rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    
def logger():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('logger', anonymous=True)
    rospy.Subscriber("task", Int8, task_callback)
    rospy.Subscriber("mecanum_motors", Int8MultiArray, value_callback)
    rospy.Subscriber("joy", Joy, joy_callback)
    r = rospy.Rate(10) #100hz
    while not rospy.is_shutdown():
        if f:
            try:
                for v in arr:
                    print >> f, str(v)+" ,",
                print >> f, "" #=\n
            except:
                pass
        r.sleep()
#    # spin() simply keeps python from exiting until this node is sto
#    rospy.spin()
        
if __name__ == '__main__':
    logger()
