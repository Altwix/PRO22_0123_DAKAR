#! /usr/bin/env python2.7

import math
import numpy
import threading

from math import pi

import rospy
import tf

# Nuevo
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped #20160509 Added for tf broadcaster
# Fin Nuevo

#20210115
from geometry_msgs.msg import Quaternion
#20210115
 
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers


class _RoadrollerCtrlr(object):
    """Ackermann controller

    An object of class _RoadrollerCtrlr is a node that controls the wheels of a
    vehicle with Ackermann steering.
    """

    def __init__(self):

        """Initialize this _RoadrollerCtrlr."""

        rospy.init_node("roadroller_controller")

        # Parameters
    
        #Distancia entre ruedas
        self.L = float(rospy.get_param("distancia_ruedas","5.85"))
        self.shock_position = float(rospy.get_param("shock_position","0.15"))
        self.model_name = rospy.resolve_name('~').split('/')[1]
        self.namespace = rospy.get_namespace()
        #print self.namespace
        #print self.model_name
        #print rospy.resolve_name('~')

        # Wheels
        (self.left_steer_link_name, left_steer_ctrlr_name,
        left_front_axle_ctrlr_name, self._left_front_inv_circ) = \
        self._get_front_wheel_params("left")

        (center_steer_link_name, center_steer_ctrlr_name,
         center_front_axle_ctrlr_name, self._center_front_inv_circ) = \
         self._get_front_wheel_params("center")

        (self.right_steer_link_name, right_steer_ctrlr_name,
         right_front_axle_ctrlr_name, self._right_front_inv_circ) = \
         self._get_front_wheel_params("right")

        (self.left_rear_link_name, left_rear_axle_ctrlr_name,
         self._left_rear_inv_circ) = \
         self._get_rear_wheel_params("left")

        (self.inner_left_rear_link_name, inner_left_rear_axle_ctrlr_name,
         self._inner_left_rear_inv_circ) = \
         self._get_rear_wheel_params("inner_left")

        (self.inner_right_rear_link_name, inner_right_rear_axle_ctrlr_name,
         self._inner_right_rear_inv_circ) = \
         self._get_rear_wheel_params("inner_right")

        (self._right_rear_link_name, right_rear_axle_ctrlr_name,
         self._right_rear_inv_circ) = \
         self._get_rear_wheel_params("right")


        list_ctrlrs = rospy.ServiceProxy("controller_manager/list_controllers",
                                         ListControllers)
        list_ctrlrs.wait_for_service()

        # Command timeout
        try:
            self._cmd_timeout = float(rospy.get_param("~cmd_timeout",
                                                      self._DEF_CMD_TIMEOUT))
        except:
            rospy.logwarn("The specified command timeout value is invalid. "
                          "The default timeout value will be used instead.")
            self._cmd_timeout = self._DEF_CMD_TIMEOUT

        # Publishing frequency
        try:
            pub_freq = float(rospy.get_param("~publishing_frequency",
                                             self._DEF_PUB_FREQ))
            if pub_freq <= 0.0:
                raise ValueError()
        except:
            rospy.logwarn("The specified publishing frequency is invalid. "
                          "The default frequency will be used instead.")
            pub_freq = self._DEF_PUB_FREQ
        self._sleep_timer = rospy.Rate(pub_freq)

        # _last_cmd_time is the time at which the most recent Ackermann
        # driving command was received.
        self._last_cmd_time = rospy.get_time()

        # _ackermann_cmd_lock is used to control access to _steer_ang,
        # _steer_ang_vel, _speed, _accel, and _jerk.
        self._ackermann_cmd_lock = threading.Lock()
        self._steer_ang = 0.0      # Steering angle
        self._steer_ang_vel = 0.0  # Steering angle velocity
        self._speed = 0.0
        self._accel = 0.0          # Acceleration
        self._jerk = 0.0

        self._last_steer_ang = 0.0  # Last steering angle
        self._theta_left = 0.0      # Left steering joint angle
        self._theta_center = 0.0    # Center steering joint angle
        self._theta_right = 0.0     # Right steering joint angle


        self._last_speed = 0.0
        self._last_accel_limit = 0.0  # Last acceleration limit
        # Axle angular velocities
        self._left_front_ang_vel = 0.0
        self._center_front_ang_vel = 0.0
        self._right_front_ang_vel = 0.0
        self._left_rear_ang_vel = 0.0
        self._inner_left_rear_ang_vel = 0.0
        self._inner_right_rear_ang_vel = 0.0
        self._right_rear_ang_vel = 0.0

		    # Nuevo
        self._position_joint = []
        self._velocity_joint = []
        self._name = []
        self._robot_pose_px_ = 0.0
        self._robot_pose_py_ = 0.0
        self._robot_pose_vx_ = 0.0
        self._robot_pose_vy_ = 0.0
        self._robot_pose_pa_ = 0.0
        self._robot_pose_va_ = 0.0
        self._robot_pose_vax_ = 0.0
        self._robot_pose_vay_ = 0.0
        self._odom = Odometry()
        self._odom_tf = TransformStamped()  #20160509 Added for tf broadcaster
        self._odom_broadcaster = tf.TransformBroadcaster(); #20160509 Added for tf broadcaster
        self._theta = 0.0
        self._pose_gse = Odometry()
        self._pose_model = Pose()
        self._twist_model = Twist()
        self._name_model = []
        self._linearSpeedXMps = 0.0
        self._linearSpeedXMps_rear = 0.0
        
 		    # Fin Nuevo

        # _joint_dist_div_2 is the distance between the steering joints,
        # divided by two.
        self._tfl = tf.TransformListener()

        ls_pos = self._get_link_pos(self._tfl, self.left_steer_link_name)
        rs_pos = self._get_link_pos(self._tfl, self.right_steer_link_name)
        self._joint_dist_div_2 = numpy.linalg.norm(ls_pos - rs_pos) / 2

        rear_ls_pos = self._get_link_pos(self._tfl, self.left_rear_link_name)
        rear_rs_pos = self._get_link_pos(self._tfl, self._right_rear_link_name)
        self._rear_joint_dist_div_2 = numpy.linalg.norm(rear_ls_pos - rear_rs_pos) / 2

        lrw_pos = self._get_link_pos(self._tfl, self.left_rear_link_name)
        rrw_pos = numpy.array([0.0] * 3)
        rear_cent_pos = (ls_pos + rs_pos) / 2     # Front center position
        front_cent_pos = (lrw_pos + rrw_pos) / 2    # Rear center position
        self._wheelbase = numpy.linalg.norm(front_cent_pos - rear_cent_pos)
        rospy.loginfo("wheelbase = %f, joint dist div_2 = %f, rear_joint_dist_div_2 = %f ",self._wheelbase,self._joint_dist_div_2,self._rear_joint_dist_div_2)
        self._inv_wheelbase = 1 / self._wheelbase  # Inverse of _wheelbase
        self._wheelbase_sqr = self._wheelbase ** 2

        # Publishers and subscribers

#        self._steer_cmd_pub = \
#            _create_cmd_pub(list_ctrlrs, steer_ctrlr_name)

        self._left_steer_cmd_pub = \
            _create_cmd_pub(list_ctrlrs, left_steer_ctrlr_name)
        self._center_steer_cmd_pub = \
            _create_cmd_pub(list_ctrlrs, center_steer_ctrlr_name)
        self._right_steer_cmd_pub = \
            _create_cmd_pub(list_ctrlrs, right_steer_ctrlr_name)
        
        rospy.loginfo("Create_cmd_pub")


        self._left_front_axle_cmd_pub = \
            _create_axle_cmd_pub(list_ctrlrs, left_front_axle_ctrlr_name)
        self._center_front_axle_cmd_pub = \
            _create_axle_cmd_pub(list_ctrlrs, center_front_axle_ctrlr_name)
        self._right_front_axle_cmd_pub = \
            _create_axle_cmd_pub(list_ctrlrs, right_front_axle_ctrlr_name)
        self._left_rear_axle_cmd_pub = \
            _create_axle_cmd_pub(list_ctrlrs, left_rear_axle_ctrlr_name)
        self._inner_left_rear_axle_cmd_pub = \
            _create_axle_cmd_pub(list_ctrlrs, inner_left_rear_axle_ctrlr_name)
        self._inner_right_rear_axle_cmd_pub = \
            _create_axle_cmd_pub(list_ctrlrs, inner_right_rear_axle_ctrlr_name)
        self._right_rear_axle_cmd_pub = \
            _create_axle_cmd_pub(list_ctrlrs, right_rear_axle_ctrlr_name)

        rospy.loginfo("Create_axle_cmd_pub")


        self._left_rear_shock_cmd_pub = rospy.Publisher("rear_left_shock_ctrlr/command", Float64,queue_size=10)
        self._inner_left_rear_shock_cmd_pub = rospy.Publisher("rear_inner_left_shock_ctrlr/command", Float64,queue_size=10)
        self._inner_right_rear_shock_cmd_pub = rospy.Publisher("rear_inner_right_shock_ctrlr/command", Float64,queue_size=10)
        self._right_rear_shock_cmd_pub = rospy.Publisher("rear_right_shock_ctrlr/command", Float64,queue_size=10)    

        self._left_front_shock_cmd_pub = rospy.Publisher("front_left_shock_ctrlr/command", Float64,queue_size=10)
        self._left_center_shock_cmd_pub = rospy.Publisher("front_center_shock_ctrlr/command", Float64,queue_size=10)
        self._right_front_shock_cmd_pub = rospy.Publisher("front_right_shock_ctrlr/command", Float64,queue_size=10)
        rospy.loginfo("Publicadores generados ackermann")

#        self._odom_pub = rospy.Publisher("/odom_dumper", Odometry)
        self._odom_pub = rospy.Publisher("odometry", Odometry,queue_size=10)
        self._pose_roadroller_pub = rospy.Publisher("pose_roadroller_gazebo",Odometry,queue_size=10)
        self._linear_vel_pub = rospy.Publisher("linear_vel",Float64,queue_size=10)
        self._linear_vel_pub_rear = rospy.Publisher("linear_vel_rear",Float64,queue_size=10)

	    # Fin Nuevo
        rospy.loginfo("Publicadores generados caja")
        

		# Nuevo
        self._ackermann_cmd_sub = \
            rospy.Subscriber("ackermann_cmd",AckermannDriveStamped,self.ackermann_cmd_cb, queue_size=1)
        self._caja_cmd_sub = \
            rospy.Subscriber("joint_states",JointState,self.joint_states_callback, queue_size=1)
        self._model_state_sub = \
            rospy.Subscriber("/gazebo/model_states",ModelStates,self.model_state_callback, queue_size=1)
                   
        rospy.loginfo("Subscripcion hecha %s", self._ackermann_cmd_sub)
	    # Fin Nuevo

    def spin(self):
        """Control the vehicle."""

        last_time = rospy.get_time()

        while not rospy.is_shutdown():
            t = rospy.get_time()
            delta_t = t - last_time
            last_time = t
#            rospy.loginfo("duracion bucle controlador = %f", delta_t)

            if (self._cmd_timeout > 0.0 and
                t - self._last_cmd_time > self._cmd_timeout):
                # Too much time has elapsed since the last command. Stop the
                # vehicle.
                rospy.loginfo("Timeout")
                steer_ang_changed, center_y = \
                    self._ctrl_steering(self._last_steer_ang, 0.0, 0.001)
                self._ctrl_axles(0.0, 0.0, 0.0, 0.001, steer_ang_changed,
                                 center_y)
            elif delta_t > 0.0:
                with self._ackermann_cmd_lock:
                    steer_ang = self._steer_ang
                    steer_ang_vel = self._steer_ang_vel
                    speed = self._speed
                    accel = self._accel
                    jerk = self._jerk      
                
                steer_ang_changed, center_y = \
                    self._ctrl_steering(steer_ang, steer_ang_vel, delta_t)

                self._ctrl_axles(speed, accel, jerk, delta_t,
                                 steer_ang_changed, center_y)

                self._update_odometry(delta_t)

            # Publish the steering and axle joint commands.
#            self._steer_cmd_pub.publish(self._theta)

            self._left_steer_cmd_pub.publish(self._theta_left)
            self._center_steer_cmd_pub.publish(self._theta_center)
            self._right_steer_cmd_pub.publish(self._theta_right)

            if self._left_front_axle_cmd_pub:
                self._left_front_axle_cmd_pub.publish(self._left_front_ang_vel)
            if self._center_front_axle_cmd_pub:
                self._center_front_axle_cmd_pub.publish(self._center_front_ang_vel)
            if self._right_front_axle_cmd_pub:
                self._right_front_axle_cmd_pub.\
                    publish(self._right_front_ang_vel)
            if self._left_rear_axle_cmd_pub:
                self._left_rear_axle_cmd_pub.publish(self._left_rear_ang_vel)
            if self._inner_left_rear_axle_cmd_pub:
                self._inner_left_rear_axle_cmd_pub.publish(self._inner_left_rear_ang_vel)
            if self._inner_right_rear_axle_cmd_pub:
                self._inner_right_rear_axle_cmd_pub.publish(self._inner_right_rear_ang_vel)
            if self._right_rear_axle_cmd_pub:
                self._right_rear_axle_cmd_pub.publish(self._right_rear_ang_vel)            
            # Nuevo
            self._left_rear_shock_cmd_pub.publish(self.shock_position)
            self._inner_left_rear_shock_cmd_pub.publish(self.shock_position)
            self._inner_right_rear_shock_cmd_pub.publish(self.shock_position)
            self._right_rear_shock_cmd_pub.publish(self.shock_position)

            self._left_front_shock_cmd_pub.publish(self.shock_position)
            self._left_center_shock_cmd_pub.publish(self.shock_position)
            self._right_front_shock_cmd_pub.publish(self.shock_position)

#            pos_front_axis_to_base_link,orient_front_axis_to_base_link = self._tfl.lookupTransform("/rear_axis","/base_link", rospy.Time())
#	    rospy.loginfo("transformation: x = %f",pos_front_axis_to_base_link[0])
#	    rospy.loginfo("transformation: y = %f",pos_front_axis_to_base_link[1])
#	    rospy.loginfo("transformation: z = %f",pos_front_axis_to_base_link[2])
#	    rospy.loginfo("rotation: x = %f",orient_front_axis_to_base_link[0])
#	    rospy.loginfo("rotation: y = %f",orient_front_axis_to_base_link[1])
#	    rospy.loginfo("rotation: z = %f",orient_front_axis_to_base_link[2])
#	    rospy.loginfo("rotation: w = %f",orient_front_axis_to_base_link[3])

            self._odom.header.stamp = rospy.Time.now()
            self._odom.header.frame_id = self.namespace + "odom"
            self._odom.child_frame_id = self.namespace + "base_link"
#            self._odom.pose.pose.position.x = self._robot_pose_px_ + pos_front_axis_to_base_link[0]*math.cos(self._robot_pose_pa_)
#            self._odom.pose.pose.position.y = self._robot_pose_py_ + pos_front_axis_to_base_link[1]*math.sin(self._robot_pose_pa_)
            self._odom.pose.pose.position.x = self._robot_pose_px_
            self._odom.pose.pose.position.y = self._robot_pose_py_
            self._odom.pose.pose.position.z = 0.0

            q = tf.transformations.quaternion_from_euler(0,0,self._robot_pose_pa_)
            self._odom.pose.pose.orientation.x = q[0]
            self._odom.pose.pose.orientation.y = q[1]
            self._odom.pose.pose.orientation.z = q[2]
            self._odom.pose.pose.orientation.w = q[3]

            for i in range (0,6):
                self._odom.pose.covariance[i*6+i] = 0.1

            self._odom.twist.twist.linear.x = self._robot_pose_vx_
            self._odom.twist.twist.linear.y = self._robot_pose_vy_
            self._odom.twist.twist.linear.z = 0.0
            self._odom.twist.twist.angular.x = 0.0
            self._odom.twist.twist.angular.y = 0.0
            self._odom.twist.twist.angular.z = self._robot_pose_va_
      
            for i in range (0,6):
                self._odom.twist.covariance[i*6+i] = 0.1

            self._odom_pub.publish(self._odom)            

	    # #20160509 Added for tf broadcaster (Luis Riazuelo riazuelo@unizar.es)
        #     position_odom_tf = (self._odom.pose.pose.position.x, \
		# 		self._odom.pose.pose.position.y, \
		# 		self._odom.pose.pose.position.z)
        #     orientation_odom_tf = (self._odom.pose.pose.orientation.x, \
		# 		self._odom.pose.pose.orientation.y, \
		# 		self._odom.pose.pose.orientation.z, \
		# 		self._odom.pose.pose.orientation.w)
        #     """ self._odom_broadcaster.sendTransform(position_odom_tf, orientation_odom_tf, \
		# 		self._odom.header.stamp, "base_link", "odom") """ #20210203: odom -> base_link tf is published by another node
        #     """ self._odom_broadcaster.sendTransform(position_odom_tf, orientation_odom_tf, \
		# 			         self._odom.header.stamp, "base_footprint", "odom") """
        #     #20160509     
            #self.get_model_state("roadroller")

            #print "Get model state ....."
            self.get_model_state(self.model_name)

            #20210115 - INI: Computation of the linear velocity of the robot from its vectorial components (vx,vy)
            twist_model_robot_frame = self.convertTwist2RobotFrame()
            #20210115 - FIN

            self._pose_gse.header.stamp = rospy.Time.now()
            self._pose_gse.header.frame_id = self.namespace + "odom" 
            self._pose_gse.child_frame_id = self.namespace + "base_link"
            self._pose_gse.pose.pose = self._pose_model        
            #20210115 - INI: Update data from the new twist value computed
            #self._pose_gse.twist.twist = self._twist_model
            self._pose_gse.twist.twist = twist_model_robot_frame
            #20210115 - FIN

            for i in range (0,6):
               self._pose_gse.pose.covariance[i*6+i] = 0.1
               
            for i in range (0,6):
               self._pose_gse.twist.covariance[i*6+i] = 0.1             

            self._pose_roadroller_pub.publish(self._pose_gse)

            self._linear_vel_pub.publish(self._linearSpeedXMps)
            self._linear_vel_pub_rear.publish(self._linearSpeedXMps_rear)
            # Fin Nuevo

            self._sleep_timer.sleep()
			

    #20200115
    def convertTwist2RobotFrame(self):  
        new_twist = Twist()
        
        angles = tf.transformations.euler_from_quaternion([self._pose_model.orientation.x, self._pose_model.orientation.y, self._pose_model.orientation.z, self._pose_model.orientation.w]) 
        yaw = angles[2]

        new_twist.linear.x = self._twist_model.linear.x*math.cos(yaw) + self._twist_model.linear.y*math.sin(yaw)
        new_twist.linear.y = 0.0
        new_twist.linear.z = 0.0

        new_twist.angular.x = 0.0
        new_twist.angular.y = 0.0
        new_twist.angular.z = self._twist_model.angular.z #new_twist.linear.x * math.tan(self._steer_ang)/self._wheelbase 
        
        return new_twist
    #20200115


	  # Nuevo		
    def joint_states_callback(self,msg):
        with self._ackermann_cmd_lock:
             self._position_joint = msg.position
             self._velocity_joint = msg.velocity
             self._name = msg.name

    def get_joint_position(self,joint_name):
        position = 0
        #joint_name = "joint_" + joint_name 

        with self._ackermann_cmd_lock:
             if joint_name in self._name:
                index = self._name.index(joint_name)
                position = self._position_joint[index]
        return position

    def get_joint_velocity(self,joint_name):
        velocity = 0  
        with self._ackermann_cmd_lock:
             if joint_name in self._name:
                index = self._name.index(joint_name)
                velocity = self._velocity_joint[index]
        return velocity

    def model_state_callback(self,msg):
        with self._ackermann_cmd_lock:
             self._position_model = msg.pose
             self._velocity_model = msg.twist
             self._name_model = msg.name

    def get_model_state(self,model_name):
        position = 0
        twist = 0
        with self._ackermann_cmd_lock:
             if model_name in self._name_model:
                index = self._name_model.index(model_name)
                self._pose_model = self._position_model[index]
                self._twist_model = self._velocity_model[index]                

 
    # Fin Nuevo	

    # Nuevo
    def _update_odometry(self,delta_t):        #CALCULO SIN GIRO DEL VOLANTE
	
       v_bl = self.get_joint_velocity("rear_left_wheel") 
       v_br = self.get_joint_velocity("rear_right_wheel")

       incremento_left = v_bl * self._DEF_WHEEL_DIA/2.0 * delta_t
       incremento_right = v_br * self._DEF_WHEEL_DIA/2.0 * delta_t

#       self._theta = self._theta + math.asin((incremento_right-incremento_left)/2.152)  
       self._linearSpeedXMps = (v_bl + v_br)*(self._DEF_WHEEL_DIA)/4.0

       self._robot_pose_va_ = math.asin((incremento_right-incremento_left)/self._DEF_REAR_AXIS_WIDE)  
       self._robot_pose_pa_ = self._robot_pose_pa_ + self._robot_pose_va_

       self._robot_pose_vx_ = self._linearSpeedXMps*math.cos(self._robot_pose_pa_)
       self._robot_pose_vy_ = self._linearSpeedXMps*math.sin(self._robot_pose_pa_)
       self._robot_pose_px_ = self._robot_pose_px_ + self._robot_pose_vx_*delta_t
       self._robot_pose_py_ = self._robot_pose_py_ + self._robot_pose_vy_*delta_t 

    # Fin Nuevo
	
	
    def ackermann_cmd_cb(self, ackermann_cmd):
        """Ackermann driving command callback

        :Parameters:
          ackermann_cmd: ackermann_msgs.msg.AckermannDriveStamped
            Ackermann driving command.
        """
        self._last_cmd_time = rospy.get_time()
        with self._ackermann_cmd_lock:
            self._steer_ang = ackermann_cmd.drive.steering_angle
            self._steer_ang_vel = ackermann_cmd.drive.steering_angle_velocity
            self._speed = ackermann_cmd.drive.speed
            self._accel = ackermann_cmd.drive.acceleration
            self._jerk = ackermann_cmd.drive.jerk

    def _get_steering_params(self, joint):
        prefix = rospy.resolve_name('~') + joint
        link_name = self.namespace + rospy.get_param(prefix + "steering_link_name",
                                          joint + "_axis")
        steer_ctrlr_name = rospy.get_param(prefix + "steering_controller_name",
                                           joint + "_ctrlr")
        #rospy.loginfo("Publicando controlador: %s",steer_ctrlr_name)
        return link_name, steer_ctrlr_name

    def _get_rear_wheel_params(self, side):
        # Get rear wheel parameters. Return a tuple containing the link name,
        # axle controller name, and inverse of the circumference.

        prefix = rospy.resolve_name('~') + "rear_" + side + "_wheel/"
        link_name = self.namespace + rospy.get_param(prefix + "link_name", side + "_wheel")
	
        axle_ctrlr_name, inv_circ = self._get_common_wheel_params(prefix)
        return link_name, axle_ctrlr_name, inv_circ

    def _get_front_wheel_params(self, side):
        # Get front wheel parameters. Return a tuple containing the steering
        # link name, steering controller name, axle controller name (or None),
        # and inverse of the circumference.

        prefix = rospy.resolve_name('~') + "front_" + side + "_wheel/"
        steer_link_name = self.namespace + rospy.get_param(prefix + "steering_link_name",
                                          side + "_steering_link")
        steer_ctrlr_name = self.namespace + rospy.get_param(prefix + "steering_controller_name",
                                           side + "_steering_controller")


        axle_ctrlr_name, inv_circ = self._get_common_wheel_params(prefix)
        return steer_link_name, steer_ctrlr_name, axle_ctrlr_name, inv_circ


    def _get_common_wheel_params(self, prefix):
        # Get parameters used by the front and rear wheels. Return a tuple
        # containing the axle controller name (or None) and the inverse of the
        # circumference.

        axle_ctrlr_name = self.namespace + rospy.get_param(prefix + "axle_controller_name",
                                          None)

        try:
            dia = float(rospy.get_param(prefix + "diameter",
                                        self._DEF_WHEEL_DIA))
            if dia <= 0.0:
                raise ValueError()
        except:
            rospy.logwarn("The specified wheel diameter is invalid. "
                          "The default diameter will be used instead.")
            dia = self._DEF_WHEEL_DIA

        return axle_ctrlr_name, 1 / (pi * dia)

    def _get_link_pos(self, tfl, link):
        # Return the position of the specified link, relative to the right
        # rear wheel link.

        #rospy.loginfo("link: %s, right_rear_link_name: %s", link, self._right_rear_link_name)

        while True:
            try:
                trans, not_used = \
                    tfl.lookupTransform(self._right_rear_link_name, link, 
                                        rospy.Time(0))
                return numpy.array(trans)
            except:
                pass

    def _ctrl_steering(self, steer_ang, steer_ang_vel_limit, delta_t):
        # Control the steering joints.

        # Compute theta, the virtual front wheel's desired steering angle.
        if steer_ang_vel_limit > 0.0:
            # Limit the sathenasteer_ang_vel_limit,
            min(ang_vel, steer_ang_vel_limit)
            theta = self._last_steer_ang + ang_vel * delta_t
        else:
            theta = steer_ang

        # Compute the desired steering angles for the left and right front
        # wheels.
#        center_y = self._wheelbase * math.tan((pi / 2) - theta)
	# saturamos theta a 45 y -45  (0.79 en radianes), lo hacemos antes del calculo del steering de cada rueda para que la exterior no pueda girar tanto como la interior
        if theta > 0.79:
            theta = 0.79
        if theta < -0.79:
            theta = -0.79

        center_y = self._wheelbase * math.tan((pi / 2) - theta)
        steer_ang_changed = theta != self._last_steer_ang
        if steer_ang_changed:
            self._last_steer_ang = theta
            self._theta_left = \
                _get_steer_ang(math.atan(self._inv_wheelbase *
                                         (center_y - self._joint_dist_div_2)))
            self._theta_right = \
                _get_steer_ang(math.atan(self._inv_wheelbase *
                                         (center_y + self._joint_dist_div_2)))
            self._theta_center = theta

        return steer_ang_changed, center_y

    def _ctrl_axles(self, speed, accel_limit, jerk_limit, delta_t,
                    steer_ang_changed, center_y):
        # Control the axle joints.

        # Compute veh_speed, the vehicle's desired speed.
        if accel_limit > 0.0:
            # Limit the vehicle's acceleration.

            if jerk_limit > 0.0:
                if self._last_accel_limit > 0.0:
                    jerk = (accel_limit - self._last_accel_limit) / delta_t
                    jerk = max(-jerk_limit, min(jerk, jerk_limit))
                    accel_limit_2 = self._last_accel_limit + jerk * delta_t
                else:
                    accel_limit_2 = accel_limit
            else:
                accel_limit_2 = accel_limit
            self._last_accel_limit = accel_limit_2

            accel = (speed - self._last_speed) / delta_t
            accel = max(-accel_limit_2, min(accel, accel_limit_2))
            veh_speed = self._last_speed + accel * delta_t
        else:
            self._last_accel_limit = accel_limit
            veh_speed = speed

        # Compute the desired angular velocities of the wheels.
        if veh_speed != self._last_speed or steer_ang_changed:
            self._last_speed = veh_speed
            left_dist = center_y - self._joint_dist_div_2
            center_dist = center_y
            right_dist = center_y + self._joint_dist_div_2

            rear_left_dist = center_y - self._rear_joint_dist_div_2
            inner_left_dist = center_y - self._rear_joint_dist_div_2/2
            inner_right_dist = center_y + self._rear_joint_dist_div_2/2
            rear_right_dist = center_y + self._rear_joint_dist_div_2

            # como ref --- center_y = self._wheelbase * math.tan((pi / 2) - theta)
            # center_y es la distancia desde el centro del eje trasero hasta el centro de
            # giro instantaneo

            # Front
            gain = (2 * pi) * veh_speed / abs(center_y)
            r = math.sqrt(left_dist ** 2 + self._wheelbase_sqr)
            self._left_front_ang_vel = gain * r * self._left_front_inv_circ

            r = math.sqrt(center_dist ** 2 + self._wheelbase_sqr)
            self._center_front_ang_vel = gain * r * self._center_front_inv_circ
     
            r = math.sqrt(right_dist ** 2 + self._wheelbase_sqr)
            self._right_front_ang_vel = gain * r * self._right_front_inv_circ

            # Rear
            gain = (2 * pi) * veh_speed / center_y # el 2*pi se va con el 2*pi de la inv de la circunferencia
            self._left_rear_ang_vel = \
                gain * rear_left_dist * self._left_rear_inv_circ
            self._inner_left_rear_ang_vel = \
                gain * inner_left_dist * self._inner_left_rear_inv_circ            
            self._inner_right_rear_ang_vel = \
                gain * inner_right_dist * self._inner_right_rear_inv_circ
            self._right_rear_ang_vel = \
                gain * rear_right_dist * self._right_rear_inv_circ

    _DEF_WHEEL_DIA = 1.338    # Default wheel diameter. Unit: meter.
    _DEF_REAR_AXIS_WIDE = 2.024 # Default rear axis wide
    _DEF_EQ_POS = 0.0       # Default equilibrium position. Unit: meter.
    _DEF_CMD_TIMEOUT = 0.5  # Default command timeout. Unit: second.
    _DEF_PUB_FREQ = 100.0    # Default publishing frequency. Unit: hertz. antes a 30.0
# end _AckermannCtrlr

def _wait_for_ctrlr(list_ctrlrs, ctrlr_name):
    # Wait for the specified controller to be in the "running" state.
    # Commands can be lost if they are published before their controller is
    # running, even if a latched publisher is used.

    while True:
        response = list_ctrlrs()
        for ctrlr in response.controller:
            #rospy.loginfo("Estado controlador (%s) %s: %s",ctrlr_name,ctrlr.name,ctrlr.state)            
            if ctrlr.name == ctrlr_name:
                #rospy.loginfo("Publicando controlador: %s",ctrlr.name)
                if ctrlr.state == "running":
                    return
                rospy.sleep(0.1)
                break


def _create_axle_cmd_pub(list_ctrlrs, axle_ctrlr_name):
    # Create an axle command publisher.    
    if not axle_ctrlr_name:
        return None
    return _create_cmd_pub(list_ctrlrs, axle_ctrlr_name)


def _create_cmd_pub(list_ctrlrs, ctrlr_name):
    # Create a command publisher.    
    _wait_for_ctrlr(list_ctrlrs, ctrlr_name)
    return rospy.Publisher(ctrlr_name + "/command", Float64,queue_size=10)


def _get_steer_ang(phi):
    # Return the desired steering angle for a front wheel.
    if phi >= 0.0:
        return (pi / 2) - phi
    return (-pi / 2) - phi


# main
if __name__ == "__main__":
    ctrlr = _RoadrollerCtrlr()
   #  rospy.loginfo("Inicializado")
    ctrlr.spin()
