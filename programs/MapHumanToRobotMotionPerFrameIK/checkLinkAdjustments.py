
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
import csv

x = []
y = []
z = []
frames = []
qx = []
qy = []
qz = []
qw = []

x_wrist = []
y_wrist = []
z_wrist = []
qx_wrist = []
qy_wrist = []
qz_wrist = []
qw_wrist = []

x_elbow = []
y_elbow = []
z_elbow = []
qx_elbow = []
qy_elbow = []
qz_elbow = []
qw_elbow = []

x_shoulder = []
y_shoulder = []
z_shoulder = []
qx_shoulder = []
qy_shoulder = []
qz_shoulder = []
qw_shoulder = []

x_neck = []
y_neck = []
z_neck = []
qx_neck = []
qy_neck = []
qz_neck = []
qw_neck = []

x_hip = []
y_hip = []
z_hip = []
qx_hip = []
qy_hip = []
qz_hip = []
qw_hip = []

x1 = []
y1 = []
z1 = []
frames1 = []
qx1 = []
qy1 = []
qz1 = []
qw1 = []

x_wrist1 = []
y_wrist1 = []
z_wrist1 = []
qx_wrist1 = []
qy_wrist1 = []
qz_wrist1 = []
qw_wrist1 = []

x_elbow1 = []
y_elbow1 = []
z_elbow1 = []
qx_elbow1 = []
qy_elbow1 = []
qz_elbow1 = []
qw_elbow1 = []

x_shoulder1 = []
y_shoulder1 = []
z_shoulder1 = []
qx_shoulder1 = []
qy_shoulder1 = []
qz_shoulder1 = []
qw_shoulder1 = []

x_neck1 = []
y_neck1 = []
z_neck1 = []
qx_neck1 = []
qy_neck1 = []
qz_neck1 = []
qw_neck1 = []

x_hip1 = []
y_hip1 = []
z_hip1 = []
qx_hip1 = []
qy_hip1 = []
qz_hip1 = []
qw_hip1 = []

def updateHumanMarkersAdj(index):
    markerArray = MarkerArray()
    marker = Marker()
    markerJoints = Marker()
    markerJoints.id = 2 
    markerJoints.header.frame_id = "waist"
    markerJoints.type = Marker.POINTS
    markerJoints.action = Marker.ADD 
    markerJoints.scale.x = 0.05
    markerJoints.scale.y = 0.05
    markerJoints.scale.z = 0.05
    markerJoints.color.a = 1.0
    markerJoints.color.b = 0.5
    markerJoints.color.r = 0.5
    markerJoints.pose.orientation.w=1.0
    markerJoints.pose.position.x = 0
    markerJoints.pose.position.y = 0
    markerJoints.pose.position.z = 0
    marker.id = 0 
    marker.header.frame_id = "waist"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD 
    marker.scale.x = 0.01
    marker.color.a = 1.0
    marker.color.b = 0.5
    marker.color.r = 0.5
    marker.pose.orientation.w=1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    
    marker.points = []
    markerJoints.points = []
        
    p_hip = Point()
    p_hip.x = x_hip[index]
    p_hip.y = y_hip[index]
    p_hip.z = z_hip[index]
    marker.points.append(p_hip)
    markerJoints.points.append(p_hip)
    
    p_neck = Point()
    p_neck.x = x_neck[index]
    p_neck.y = y_neck[index]
    p_neck.z = z_neck[index]
    marker.points.append(p_neck)
    markerJoints.points.append(p_neck)
    
    p = Point()
    p.x = x_shoulder[index]
    p.y = y_shoulder[index]
    p.z = z_shoulder[index]
    print("p.x: ", p.x, "p.y: ", p.y, "p.z: ", p.z)
    marker.points.append(p)
    markerJoints.points.append(p)
    
    p1 = Point()
    p1.x = x_elbow[index]
    p1.y = y_elbow[index]
    p1.z = z_elbow[index]
    marker.points.append(p1)
    markerJoints.points.append(p1)
    print("p1.x: ", p1.x, "p1.y: ", p1.y, "p1.z: ", p1.z)
    
    p2 = Point()
    p2.x = x_wrist[index]
    p2.y = y_wrist[index]
    p2.z = z_wrist[index]
    print("p2.x: ", p2.x, "p2.y: ", p2.y, "p2.z: ", p2.z)
    marker.points.append(p2)
    markerJoints.points.append(p2)

    p3 = Point()
    p3.x = x[index]
    p3.y = y[index]
    p3.z = z[index]
    print("p3.x: ", p3.x, "p3.y: ", p3.y, "p3.z: ", p3.z)
    marker.points.append(p3)
    markerJoints.points.append(p3)

    markerArray.markers.append(marker)#add linestrip to markerArray
    markerArray.markers.append(markerJoints)#add linestrip to markerArray
    
    return markerArray

def updateHumanMarkers(index):
    trunkHeight = z_hip1[index]
    markerArray1 = MarkerArray()
    marker1 = Marker()
    markerJoints1 = Marker()
    markerJoints1.id = 3 
    markerJoints1.header.frame_id = "waist_pitch_link"
    markerJoints1.type = Marker.POINTS
    markerJoints1.action = Marker.ADD 
    markerJoints1.scale.x = 0.05
    markerJoints1.scale.y = 0.05
    markerJoints1.scale.z = 0.05
    markerJoints1.color.a = 1.0
    markerJoints1.color.b = 0.0
    markerJoints1.color.r = 1.0
    markerJoints1.pose.orientation.w=1.0
    markerJoints1.pose.position.x = 0
    markerJoints1.pose.position.y = 0
    markerJoints1.pose.position.z = 0
    marker1.id = 0 
    marker1.header.frame_id = "waist_pitch_link"
    marker1.type = Marker.LINE_STRIP
    marker1.action = Marker.ADD 
    marker1.scale.x = 0.01
    marker1.color.a = 1.0
    marker1.color.b = 0.0
    marker1.color.r = 1.0
    marker1.pose.orientation.w=1.0
    marker1.pose.position.x = 0
    marker1.pose.position.y = 0
    marker1.pose.position.z = 0
    
    marker1.points = []
    markerJoints1.points = []
        
    p_hip1 = Point()
    p_hip1.x = x_hip1[index]-x_hip1[index]
    p_hip1.y = y_hip1[index]-y_hip1[index]
    p_hip1.z = z_hip1[index]-z_hip1[index]
    marker1.points.append(p_hip1)
    markerJoints1.points.append(p_hip1)
    
    p_neck1 = Point()
    p_neck1.x = x_neck1[index]-x_hip1[index]
    p_neck1.y = y_neck1[index]-y_hip1[index]
    p_neck1.z = z_neck1[index]-trunkHeight
    marker1.points.append(p_neck1)
    markerJoints1.points.append(p_neck1)
    
    p = Point()
    p.x = x_shoulder1[index]-x_hip1[index]
    p.y = y_shoulder1[index]-y_hip1[index]
    p.z = z_shoulder1[index]-trunkHeight
    print("p.x: ", p.x, "p.y: ", p.y, "p.z: ", p.z)
    marker1.points.append(p)
    markerJoints1.points.append(p)
    
    p1 = Point()
    p1.x = x_elbow1[index]-x_hip1[index]
    p1.y = y_elbow1[index]-y_hip1[index]
    p1.z = z_elbow1[index]-trunkHeight
    marker1.points.append(p1)
    markerJoints1.points.append(p1)
    print("p1.x: ", p1.x, "p1.y: ", p1.y, "p1.z: ", p1.z)
    
    p2 = Point()
    p2.x = x_wrist1[index]-x_hip1[index]
    p2.y = y_wrist1[index]-y_hip1[index]
    p2.z = z_wrist1[index]-trunkHeight
    print("p2.x: ", p2.x, "p2.y: ", p2.y, "p2.z: ", p2.z)
    marker1.points.append(p2)
    markerJoints1.points.append(p2)

    p3 = Point()
    p3.x = x1[index]-x_hip1[index]
    p3.y = y1[index]-y_hip1[index]
    p3.z = z1[index]-trunkHeight
    print("p3.x: ", p3.x, "p3.y: ", p3.y, "p3.z: ", p3.z)
    marker1.points.append(p3)
    markerJoints1.points.append(p3)

    markerArray1.markers.append(marker1)#add linestrip to markerArray
    markerArray1.markers.append(markerJoints1)#add linestrip to markerArray
    
    return markerArray1

initPathFile ="/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba1-smoothed-link-adj.csv"
csvFile = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba1-smoothed.csv";

with open(initPathFile) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=' ')
    line_count = 0
    for row in csv_reader:
        x.append(float(row[1]))  
        y.append(float(row[2]))
        z.append(float(row[3]))
            
        qw.append(float(row[7]))
        qx.append(float(row[4]))
        qy.append(float(row[5]))
        qz.append(float(row[6]))
            
            
        # #wrist position+orientation
            
        x_wrist.append(float(row[8]))
        y_wrist.append(float(row[9]))
        z_wrist.append(float(row[10]))
        qx_wrist.append(float(row[11]))
        qy_wrist.append(float(row[12]))
        qz_wrist.append(float(row[13]))
        qw_wrist.append(float(row[14]))
            
        # #elbow position+orientation
            
        x_elbow.append(float(row[15]))
        y_elbow.append(float(row[16]))
        z_elbow.append(float(row[17]))
        qx_elbow.append(float(row[18]))
        qy_elbow.append(float(row[19]))
        qz_elbow.append(float(row[20]))
        qw_elbow.append(float(row[21]))
            
        x_shoulder.append(float(row[22]))
        y_shoulder.append(float(row[23]))
        z_shoulder.append(float(row[24]))
        qx_shoulder.append(float(row[25]))
        qy_shoulder.append(float(row[26]))
        qz_shoulder.append(float(row[27]))
        qw_shoulder.append(float(row[28]))
        
        x_neck.append(float(row[29]))
        y_neck.append(float(row[30]))
        z_neck.append(float(row[31]))
        qx_neck.append(float(row[32]))
        qy_neck.append(float(row[33]))
        qz_neck.append(float(row[34]))
        qw_neck.append(float(row[35]))
        
        x_hip.append(float(row[36]))
        y_hip.append(float(row[37]))
        z_hip.append(float(row[38]))
        qx_hip.append(float(row[39]))
        qy_hip.append(float(row[40]))
        qz_hip.append(float(row[41]))
        qw_hip.append(float(row[42]))

        line_count += 1

        print(f'Processed {line_count} lines.')


with open(csvFile) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        x1.append(float(row[1]))  
        y1.append(float(row[2]))
        z1.append(float(row[3]))
            
        qw1.append(float(row[7]))
        qx1.append(float(row[4]))
        qy1.append(float(row[5]))
        qz1.append(float(row[6]))
            
            
        # #wrist position+orientation
            
        x_wrist1.append(float(row[8]))
        y_wrist1.append(float(row[9]))
        z_wrist1.append(float(row[10]))
        qx_wrist1.append(float(row[11]))
        qy_wrist1.append(float(row[12]))
        qz_wrist1.append(float(row[13]))
        qw_wrist1.append(float(row[14]))
            
        # #elbow position+orientation
            
        x_elbow1.append(float(row[15]))
        y_elbow1.append(float(row[16]))
        z_elbow1.append(float(row[17]))
        qx_elbow1.append(float(row[18]))
        qy_elbow1.append(float(row[19]))
        qz_elbow1.append(float(row[20]))
        qw_elbow1.append(float(row[21]))
            
        x_shoulder1.append(float(row[22]))
        y_shoulder1.append(float(row[23]))
        z_shoulder1.append(float(row[24]))
        qx_shoulder1.append(float(row[25]))
        qy_shoulder1.append(float(row[26]))
        qz_shoulder1.append(float(row[27]))
        qw_shoulder1.append(float(row[28]))
        
        x_neck1.append(float(row[29]))
        y_neck1.append(float(row[30]))
        z_neck1.append(float(row[31]))
        qx_neck1.append(float(row[32]))
        qy_neck1.append(float(row[33]))
        qz_neck1.append(float(row[34]))
        qw_neck1.append(float(row[35]))
        
        x_hip1.append(float(row[36]))
        y_hip1.append(float(row[37]))
        z_hip1.append(float(row[38]))
        qx_hip1.append(float(row[39]))
        qy_hip1.append(float(row[40]))
        qz_hip1.append(float(row[41]))
        qw_hip1.append(float(row[42]))

        line_count += 1

        print(f'Processed {line_count} lines.')


rospy.init_node('check_link_adjustments', anonymous=True)
rate = rospy.Rate(10)
pub_marker_array_adj = rospy.Publisher('link_lenght_adjusted_markers', MarkerArray, queue_size=10)
pub_marker_array = rospy.Publisher('human_markers', MarkerArray, queue_size=10)
index = 0
while not rospy.is_shutdown():
    if(index < len(x)):
        print("index: ", index, "len(x): ", len(x))
        markerArray1 = updateHumanMarkers(index)
        markerArray = updateHumanMarkersAdj(index)
        print("Publish markers")
        pub_marker_array_adj.publish(markerArray)
        pub_marker_array.publish(markerArray1)
        print("Markers published")
        # rospy.spin()
        # print("Spin")
        index+=1
    rate.sleep()
    print("Sleep")
   