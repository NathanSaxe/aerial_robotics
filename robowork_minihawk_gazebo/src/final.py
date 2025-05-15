#!/usr/bin/python
import rospy
from mavros_msgs.srv import SetMode, CommandBool
from apriltag_ros.msg import AprilTagDetectionArray
from mavros_msgs.msg import OverrideRCIn


class final:
    def __init__(self):
        self.apriltagData = None
        self.apriltagDetection = False
        self.set_auto_mode()
        self.arm_motors()
        self.wait_for_apriltag()
        
        #initialize the node, set anonymous to true
        rospy.init_node('newNode', anonymous = True)
        
        #establishing services
        rospy.wait_for_service('/minihawk_SIM/mavros/set_mode')
        rospy.wait_for_service('/minihawk_SIM/mavros/cmd/arming')

        self.set_loiter_mode()
        publish_control = rospy.Publisher('/minihawk_SIM/mavros/rc/override', OverrideRCIn, queue_size = 10)
        kRho = 10.0
        Ki = 0.1
        Kd = 5.0
        integral_x = integral_y = prev_error_x = prev_error_y = 0.0

        last_time = rospy.Time.now().to_sec()
        while True:
            if self.apriltagData:
                apriltag_position = self.get_apriltag_position(self.apriltagData.pose)
                apriltag_x_offset = apriltag_position.x
                apriltag_y_offset = apriltag_position.y
                current_time = rospy.Time.now().to_sec()
                timeDifference = max(current_time - last_time, 1e-3) 
                last_time = current_time
                prev_error_y = apriltag_y_offset
                prev_error_x = apriltag_x_offset

                throttle = 1500
                roll = compute_rot(apriltag_y_offset, prev_error_y, timeDifference, kRho, Ki, Kd)
                pitch = compute_rot(apriltag_x_offset, prev_error_x, timeDifference, kRho, Ki, Kd)
                yaw = 1500

                control = OverrideRCIn()
                control.channels = [roll, pitch, throttle, yaw, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

                publish_control.publish(control)

                rospy.sleep(0.5)

            self.set_land_mode()
    
    def arm_motors(self):
        armed = rospy.ServiceProxy('/minihawk_SIM/mavros/cmd/arming', CommandBool)
        armed(True)

    def compute_rot(self, offset, prevError, timeDiff, rho, theta, phi):
        integral += offset * timeDiff
        derivative = (offset - prevError)/timeDiff
        output = rho * offset + theta * integral + phi * derivative 
        return int(max(1000, min(2000, 1500 + output)))
    
    def apriltag_return(self, message):
        if len(message.detections) > 0:
            self.apriltagData = message.detections[0]
            self.apriltagDetection = True
    def wait_for_apriltag(self):
        apriltag_subscriber = rospy.Subscriber('/minihawk_SIM/MH_usb_camera_link_optical/tag_detections', AprilTagDetectionArray, self.apriltag_return)
        while not self.apriltagDetection and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.sleep(3)
        return apriltag_subscriber
    def get_apriltag_position(self, pose):
        while hasattr(pose, "pose"):
            pose = pose.pose
        return pose.position
        
    def set_land_mode(self):
        set_mode = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
        set_mode(0, 'QLAND')
    def set_auto_mode(self):
        set_mode = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
        set_mode(0, 'AUTO')
    def set_loiter_mode(self):
        set_mode = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
        set_mode(0, 'QLOITER')
    
program = final()