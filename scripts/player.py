#!/usr/bin/env python
# license removed for brevity
import rospy
import csv
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray
from conf import Tasks
import os

task_pub = rospy.Publisher('task', Int8, queue_size=10)
fileName = "2018-04-08-20-20-31.txt"#"2018-04-08-20-16-37.txt"#"odom.txt"
lines = []
arr = []
i = 0

def task_callback(data):    
    r = data.data
    if r == Tasks.ODOMRUN_BEGIN_STARTTOPASS1:
        print 'start mechanum play STARTTOPASS1'
        #f = open('odom.txt', 'a') # xxx rename...
        loadFile('odom.txt')
        
def loadFile(fileName):
    global f
    global lines
    global i
    f = open(os.path.expanduser(str('~/catkin_ws/src/ar_main/scripts/'+fileName)), 'r')
    lines = list(csv.reader(f))
    f.close()
    i = 0

def player():
    pub = rospy.Publisher('mecanum_motors', Int8MultiArray, queue_size=10)
    rospy.init_node('player', anonymous=True)
    r = rospy.Rate(10) # 10hz
    loadFile(fileName)
    global i
    global lines
#    print lines
    print len(lines)
    print i
    while not rospy.is_shutdown():
        if lines:
            if i < len(lines):
                print 'odom_run: ', 
                line = lines[i]
                print line
                arr = []
                for j in range(0, 4):
                    arr.append(int(line[j]))
                pub.publish(Int8MultiArray(data=arr))
                i+=1
            elif i == len(lines):
                #finish
                task_pub.publish(Int8(Tasks.ODOMRUN_END))
                i+=1
            else:
                pass
                                
        #str = "hello world %s"%rospy.get_time()
        #rospy.loginfo(str)
        #pub.publish(str)
        r.sleep()

def load():
    pass
        
if __name__ == '__main__':
    try:
        player()
    except rospy.ROSInterruptException:
        pass
