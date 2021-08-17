
import rclpy
from rclpy.node import Node
    
from geometry_msgs.msg import Twist


import roboclaw2.roboclaw as roboclaw
import math
import roboclaw2.odom_pub as odom_pub

from roboclaw2_msgs.msg import RoboClawStatus

import numpy as np

"""
Drive from commandline:
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.5}}'
"""

class RoboClaw2(Node):


    def __init__(self):
        super().__init__('roboclaw2_node')

        self.declare_parameter("com_port","/dev/ttyACM0")
        self.declare_parameter("baud_rate",230400)

        self.declare_parameter("roboclaw_address",128)
        self.declare_parameter("lin_dir",1.0)
        self.declare_parameter("ang_dir",1.0)
        
        self.declare_parameter("base_width",0.5)
        self.declare_parameter("wheel_radius",0.05)
        self.declare_parameter("encoder_resolution",4096.0)
        
        self.declare_parameter("publish_tf",False)
        self.declare_parameter("publish_joint_states",False)
        
        self.declare_parameter("left_joint_name","left_wheel_joint")
        self.declare_parameter("right_joint_name","right_wheel_joint")

        self.declare_parameter("odom_frame","odom")
        self.declare_parameter("base_frame","base_footprint")



        self.com_port=self.get_parameter("com_port").get_parameter_value().string_value
        self.baud_rate=self.get_parameter("baud_rate").get_parameter_value().integer_value

        self.rc_address=self.get_parameter("roboclaw_address").get_parameter_value().integer_value
        self.lin_dir=self.get_parameter("lin_dir").get_parameter_value().double_value
        self.ang_dir=self.get_parameter("ang_dir").get_parameter_value().double_value
        
        self.base_width=self.get_parameter("base_width").get_parameter_value().double_value
        self.wheel_radius=self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.encoder_res=self.get_parameter("encoder_resolution").get_parameter_value().double_value
        
        self.pub_tf=self.get_parameter("publish_tf").get_parameter_value().bool_value
        self.pub_joint_states=self.get_parameter("publish_joint_states").get_parameter_value().bool_value

        self.left_joint=self.get_parameter("left_joint_name").get_parameter_value().string_value
        self.right_joint=self.get_parameter("right_joint_name").get_parameter_value().string_value
        
        odom_frame=self.get_parameter("odom_frame").get_parameter_value().string_value
        base_frame=self.get_parameter("base_frame").get_parameter_value().string_value


        self.get_logger().info ("got param com_port=%s" % self.com_port)
        self.get_logger().info ("got param baud_rate=%d" % self.baud_rate)
        self.get_logger().info ("got param roboclaw_address=%d" % self.rc_address)
        self.get_logger().info ("got param lin_dir=%f" % self.lin_dir)
        self.get_logger().info ("got param ang_dir=%f" % self.ang_dir)
        self.get_logger().info ("got param base_width=%f" % self.base_width)
        self.get_logger().info ("got param wheel_radius=%f" % self.wheel_radius)
        self.get_logger().info ("got param encoder_res=%f" % self.encoder_res)
        self.get_logger().info ("got param odom_frame=%s" % odom_frame)
        self.get_logger().info ("got param base_frame=%s" % base_frame)
        self.get_logger().info ("got param publish_tf=%s" % self.pub_tf)
        self.get_logger().info ("got param publish_joint_states=%s" % self.pub_joint_states)

        if(self.pub_joint_states):
            self.get_logger().info ("got param left_joint_name=%s" % self.left_joint)
            self.get_logger().info ("got param right_joint_name=%s" % self.right_joint)


        # scale linear distance to encoder counts
        self.vel_scale = self.encoder_res / (2.0 * math.pi * self.wheel_radius)
        
        if not roboclaw.Open(self.com_port,self.baud_rate):  # baud does not matter for USB
            self.get_logger().error("Failed to open serial port: '%s'",self.com_port)

        self.last_sent_time = self.get_clock().now()

        self.cmd_vel_sub = self.create_subscription(Twist,"cmd_vel", self.callback,10)
        self.odom_pub = odom_pub.OdomPub(self,self.base_width,odom_frame,base_frame,self.pub_tf,self.pub_joint_states,self.left_joint,self.right_joint)
        
        self.mtr_status_pub = self.create_publisher(RoboClawStatus,"mtr_status", 50)
        self.got_cmd_vel = False

        self.initRoboClaw()

        timer_period = 1.0/40.0  # seconds
        self.timer = self.create_timer(timer_period, self.run)

        self.get_logger().info("Roboclaw2 Started")
        
        
    def initRoboClaw(self):
        qpps = 14500
        P=0x120
        I=0x030
        D=0x00
        
        #roboclaw.SetConfig(self.rc_address,0x3 + 0xA0 + 0x10) 
        #roboclaw.SetMainVoltages(self.rc_address,90,140)
        #roboclaw.SetLogicVoltages(self.rc_address,110,130)
        
        #roboclaw.SetM1EncoderMode(self.rc_address,0x0)
        #roboclaw.SetM2EncoderMode(self.rc_address,0x0)
        
        #roboclaw.SetPWMMode(self.rc_address,1)
        
        #roboclaw.SetM1MaxCurrent(self.rc_address,500)
        #roboclaw.SetM2MaxCurrent(self.rc_address,500)
        
        #roboclaw.SetM1VelocityPID(self.rc_address,P,I,D,qpps)
        #roboclaw.SetM2VelocityPID(self.rc_address,P,I,D,qpps)
        
        self.mc_version = "Unknown"
        mc_tp = roboclaw.ReadVersion(self.rc_address)
        if mc_tp[0]==1:
            self.mc_version = mc_tp[1]

        self.get_logger().info("Roboclaw has Firmware version: %s"% self.mc_version)
        #roboclaw.WriteNVM(address)
    


    def run(self):

        now = self.get_clock().now()

        try:
            if self.got_cmd_vel:
                self.got_cmd_vel = False
                #print("send speed %d,%d" % (int(self.Pl),int(self.Pr)))
                roboclaw.SpeedM1M2(self.rc_address,int(self.Pl),int(self.Pr))
                self.last_sent_time = self.get_clock().now()
		  
        except Exception as ex:
            self.get_logger().info( "Exception sending velocity: %s" % ex)


        if (now - self.last_sent_time).nanoseconds/1.0e9 > 0.25:
            try:
                #print("send speed 0,0")
                roboclaw.SpeedM1M2(self.rc_address,0,0)
                self.last_sent_time = now
            except  Exception as ex:
                self.get_logger().info( "Exception publishing 0 velocity: %s" % ex)

        try:

            s1 = roboclaw.ReadSpeedM1(self.rc_address)
            s2 = roboclaw.ReadSpeedM2(self.rc_address)
            e1 = roboclaw.ReadEncM1(self.rc_address)
            e2 = roboclaw.ReadEncM2(self.rc_address)
            
            dir1 = 1 if s1[2]==0 else -1
            dir2 = 1 if s2[2]==0 else -1

            cnt_l = dir1*s1[1]*self.lin_dir
            cnt_r = dir2*s2[1]*self.lin_dir

            al=self.enc_to_angle(e1)
            ar=self.enc_to_angle(e2)

            self.odom_pub.publishOdom(cnt_l/self.vel_scale,cnt_r/self.vel_scale,al,ar)

        except Exception as ex:
            self.get_logger().info( "Exception publishing ODOM: %s" % ex)


        try:

            mc_sts=-1
            mc_tp = roboclaw.ReadError(self.rc_address)
            if mc_tp[0]==1:
                mc_sts = mc_tp[1]


            
            mc_bat_volt=-1.0
            mc_tp = roboclaw.ReadMainBatteryVoltage(self.rc_address)
            if mc_tp[0]==1:
                mc_bat_volt = mc_tp[1]/10.0
            
            
            mc_logic_volt=-1.0
            mc_tp = roboclaw.ReadLogicBatteryVoltage(self.rc_address)
            if mc_tp[0]==1:
                mc_logic_volt = mc_tp[1]/10.0
            
            mc_temp_1=-1.0
            mc_tp = roboclaw.ReadTemp(self.rc_address)
            if mc_tp[0]==1:
                mc_temp_1 = mc_tp[1]/10.0
            
            mc_temp_2=-1.0
            mc_tp = roboclaw.ReadTemp2(self.rc_address)
            if mc_tp[0]==1:
                mc_temp_2 = mc_tp[1]/10.0

        except Exception as ex:
            self.get_logger().info( "Exception reading roboclaw: %s" % ex)

        
        try:
            rcs=RoboClawStatus()

            rcs.status=mc_sts
            rcs.version=self.mc_version
            rcs.motor_batt_volt=mc_bat_volt
            rcs.logic_batt_volt=mc_logic_volt
            rcs.temp1=mc_temp_1
            rcs.temp2=mc_temp_2

            self.mtr_status_pub.publish(rcs)
        except Exception as ex:
            self.get_logger().info ("Exception publishing motor status: %s" % ex)

    def check_enc_over_flow(self,enc):
        dir = 1 if (enc[2] & 2) == 0 else -1
        max_of = np.int64(4294967296)

        p = np.int64(np.uint32(enc[1]))

        if dir == 1:
            if enc[2] & 5 == 1:
               p += max_of

        if dir == -1:
            if enc[2] & 5 == 4:
                p -= max_of

        return p

    def enc_to_angle(self,enc):
        p=self.check_enc_over_flow(enc)

        (d,r) = divmod(p,np.int64(self.encoder_res))

        revolutions = 2.0 * math.pi * ( np.float64(d) + np.float64(r)/self.encoder_res)

        return math.atan2(math.sin(revolutions),math.cos(revolutions))

    def callback(self,twist):

       
        #print ("got twist",twist)
        self.Pr = (self.lin_dir*twist.linear.x + self.ang_dir*twist.angular.z*self.base_width/2.0) * self.vel_scale
        self.Pl = (self.lin_dir*twist.linear.x - self.ang_dir*twist.angular.z*self.base_width/2.0) * self.vel_scale
        self.got_cmd_vel = True

       
def main(args=None):
    rclpy.init(args=args)

    rc = RoboClaw2()
    rclpy.spin(rc)
    roboclaw.Close()
    rc.get_logger().info("roboclaw2 exiting...")
    rc.destroy_node()

    rclpy.shutdown()
    

if __name__ == "__main__":
   main()

   

