#!/usr/bin/python

import rospy
from math import sin,cos,sqrt,atan2,acos,pi
import numpy as np
import threading
from scipy.optimize import minimize
from crazyflie_driver.msg import VelocityWorld
from optimal_sensing.msg import output_uavs_pose
from optimal_sensing.msg import output_uavs_controller
from optimal_sensing.msg import output_ukf
from optimal_sensing.msg import output_FOV
from optimal_sensing.msg import output_tracking_distance
from optimal_sensing.msg import output_inter_uavs_distance
from optimal_sensing.msg import output_occlusion
from std_msgs.msg import Float64MultiArray

fx,fy,lx,ly = 381,381,640,480
target_length,target_width,target_height = 0.138,0.178,0.16
Pc,Pr,Pb,theta_c,u_desired,A,b = None,None,None,None,None,None,None
Pq = np.array([0, 0, target_height/2])
Vq = np.array([0, 0, 0]) 
uavs_pose_flag = False
ukf_flag = False
uavs_controller_flag = False
optimal_sensing_mode = False
tracking_distance = output_tracking_distance()
FOV = output_FOV()
inter_uavs_distance = output_inter_uavs_distance()
occlusion = output_occlusion()
camera_cmd = VelocityWorld()
ranging_cmd = VelocityWorld()
bearing_cmd = VelocityWorld()
theta_fov_fix = 30
theta_occ = 15
d_safe_car = 0.6
d_measuring = 2.0
d_safe_uav = 0.3
gamma_safe_car = 1
gamma_measuring = 1
gamma_safe_uav = 1
gamma_fov = 1
gamma_occ = 1

def object_fun(x):
        return np.linalg.norm(np.array(x[:12])-u_desired)

def cons_maker(i=0):
	def constraint(x):
		return b[i] - A[i,0]*x[0] - A[i,1]*x[1] - A[i,2]*x[2] - A[i,3]*x[3] - A[i,4]*x[4] - A[i,5]*x[5] - A[i,6]*x[6] - A[i,7]*x[7] - A[i,8]*x[8] - A[i,9]*x[9] -A[i,10]*x[10]-A[i,11]*x[11]- x[i+12]
	return constraint

def cons_maker1(i=0):
	def constraint(x):
		return x[i+12]
	return constraint

def uavs_pose_cb(msg):
	global Pc,Pr,Pb,theta_c,uavs_pose_flag
	Pc = np.array([msg.rx_c.data, msg.ry_c.data, msg.rz_c.data])
	Pr = np.array([msg.rx_r.data, msg.ry_r.data, msg.rz_r.data])
	Pb = np.array([msg.rx_b.data, msg.ry_b.data, msg.rz_b.data])
	theta_c = msg.theta_c.data
	if msg != None:
		uavs_pose_flag = True
	

def ukf_cb(msg):
	global Pq,Vq,optimal_sensing_mode,ukf_flag
	Pq = np.array([msg.target_pose.x, msg.target_pose.y, target_height/2])
	Vq = np.array([msg.target_vel.x, msg.target_vel.y, 0])
	optimal_sensing_mode = msg.cmode.data
	if msg != None:
		ukf_flag = True
	

def uavs_controller_cb(msg):
	global u_desired,uavs_controller_flag
	u_desired = np.array([msg.vx_1.data, msg.vy_1.data, msg.vz_1.data,msg.vx_2.data, msg.vy_2.data, msg.vz_2.data,msg.vx_3.data, msg.vy_3.data, msg.vz_3.data,msg.wz_1.data, msg.wz_2.data, msg.wz_3.data])
	if msg != None:
		uavs_controller_flag = True

def CBF():
	global A,b,inter_uavs_distance,FOV,occlusion
	nc = np.array([cos(theta_c),sin(theta_c),0])
	nc_dot = np.array([-sin(theta_c),cos(theta_c),0])
	r_cq= np.array([Pc[0] - Pq[0],Pc[1] - Pq[1],Pc[2] - Pq[2]])
	r_cq_xy= np.array([Pc[0] - Pq[0],Pc[1] - Pq[1],0])
	r_rq= np.array([Pr[0] - Pq[0],Pr[1] - Pq[1],Pr[2] - Pq[2]])
	r_bq= np.array([Pb[0] - Pq[0],Pb[1] - Pq[1],Pb[2] - Pq[2]])
	r_qc = -r_cq
	r_qc_xy = -r_cq_xy
	r_qr = -r_rq
	r_qb = -r_bq
	r_qc_norm = np.linalg.norm(r_qc)
	r_qr_norm = np.linalg.norm(r_qr)
	r_qb_norm = np.linalg.norm(r_qb)
	r_cr= np.array([Pc[0] - Pr[0],Pc[1] - Pr[1],Pc[2] - Pr[2]])
	r_cb= np.array([Pc[0] - Pb[0],Pc[1] - Pb[1],Pc[2] - Pb[2]])
	r_rb= np.array([Pr[0] - Pb[0],Pr[1] - Pb[1],Pr[2] - Pb[2]])
	r_rc = -r_cr
	r_bc = -r_cb
	r_br = -r_rb
	r_cr_norm = np.linalg.norm(r_cr)
	r_cb_norm = np.linalg.norm(r_cb)
	r_rb_norm = np.linalg.norm(r_rb)
	r_rc_norm = np.linalg.norm(r_rc)
	r_bc_norm = np.linalg.norm(r_bc)
	r_br_norm = np.linalg.norm(r_br)
	tracking_distance.rho_1.data = r_qc_norm
	tracking_distance.rho_2.data = r_qr_norm
	tracking_distance.rho_3.data = r_qb_norm
	tracking_distance_pub.publish(tracking_distance)
	inter_uavs_distance.rho_12.data = r_cr_norm
	inter_uavs_distance.rho_13.data = r_cb_norm
	inter_uavs_distance.rho_23.data = r_br_norm
	inter_uavs_distance_pub.publish(inter_uavs_distance)
	theta_qc = acos(np.dot(nc,r_qc)/r_qc_norm)
	FOV.theta_qc.data = (180*theta_qc)/pi
	#rho_cq_xy = sqrt((Pc[0] - Pq[0])**2+(Pc[1] - Pq[1])**2)
	#alpha_1 = acos(np.dot(nc,r_qc_xy)/np.linalg.norm(r_qc_xy))
	#theta_fov_h = atan2(((rho_cq_xy-target_width/2+target_length/2)*lx)/((rho_cq_xy-target_width/2-target_length/2)*2*fx))
	#theta_fov_v = atan2(((rho_cq_xy-sqrt(target_width**2+target_length**2))*ly)/(rho_cq_xy*2*fy)-(target_height/(2*rho_cq_xy*alpha_1)))
	#theta_fov_max = min(theta_fov_h,theta_fov_v)
	#FOV.theta_fov_max.data = (180*theta_fov_max)/pi
	theta_FOV_pub.publish(FOV)
	theta_occ_cr = (180*acos(np.dot(r_rc,r_qc)/r_rc_norm/r_qc_norm)/pi)
	theta_occ_cb = (180*acos(np.dot(r_bc,r_qc)/r_bc_norm/r_qc_norm)/pi)
	theta_occ_rc = (180*acos(np.dot(r_cr,r_qr)/r_cr_norm/r_qr_norm)/pi)
	theta_occ_rb = (180*acos(np.dot(r_br,r_qr)/r_br_norm/r_qr_norm)/pi)
	theta_occ_bc = (180*acos(np.dot(r_cb,r_qb)/r_cb_norm/r_qb_norm)/pi)
	theta_occ_br = (180*acos(np.dot(r_rb,r_qb)/r_rb_norm/r_qb_norm)/pi)
	occlusion.theta_occ_cr.data = theta_occ_cr)
	occlusion.theta_occ_cb.data = theta_occ_cb)
	occlusion.theta_occ_rc.data = theta_occ_rc)
	occlusion.theta_occ_rb.data = theta_occ_rb)
	occlusion.theta_occ_bc.data = theta_occ_bc)
	occlusion.theta_occ_br.data = theta_occ_br)
	theta_occ_pub.publish(occlusion)
	A = np.array([[-2*r_cq[0], -2*r_cq[1], -2*r_cq[2]]+[0]*9, \
				  [0]*3+[-2*r_rq[0], -2*r_rq[1], -2*r_rq[2]]+[0]*6, \
				  [0]*6+[-2*r_bq[0], -2*r_bq[1], -2*r_bq[2]]+[0]*3, \
				  [2*r_cq[0], 2*r_cq[1], 2*r_cq[2]]+[0]*9, \
				  [0]*3+[2*r_rq[0], 2*r_rq[1], 2*r_rq[2]]+[0]*6, \
				  [0]*6+[2*r_bq[0], 2*r_bq[1], 2*r_bq[2]]+[0]*3, \
				  [-2*r_cr[0], -2*r_cr[1], -2*r_cr[2]]+[0]*9, \
				  [-2*r_cb[0], -2*r_cb[1], -2*r_cb[2]]+[0]*9, \
				  [0]*3+[-2*r_rc[0], -2*r_rc[1], -2*r_rc[2]]+[0]*6, \
				  [0]*3+[-2*r_rb[0], -2*r_rb[1], -2*r_rb[2]]+[0]*6, \
				  [0]*6+[-2*r_bc[0], -2*r_bc[1], -2*r_bc[2]]+[0]*3, \
				  [0]*6+[-2*r_br[0], -2*r_br[1], -2*r_br[2]]+[0]*3, \
				  np.append(np.append(-(np.dot(nc,r_qc)*r_qc/r_qc_norm**3-nc/r_qc_norm)/sqrt(1 - np.dot(nc,r_qc)**2/r_qc_norm**2),[0]*6),np.append(-np.dot(nc_dot,r_qc)/r_qc_norm/sqrt(1 - np.dot(nc,r_qc)**2/r_qc_norm**2),[0]*2)), \
				  np.append(np.append(-((r_qc-r_rc)/r_rc_norm/r_qc_norm+(np.dot(r_rc,r_qc)*r_rc)/r_rc_norm**3/r_qc_norm+(np.dot(r_rc,r_qc)*r_qc)/r_rc_norm/r_qc_norm**3)/sqrt(1 - np.dot(r_rc,r_qc)**2/r_rc_norm**2/r_qc_norm**2), \
				  -(r_qc/r_rc_norm/r_qc_norm-(np.dot(r_rc,r_qc)*r_rc)/r_rc_norm**3/r_qc_norm)/sqrt(1 - np.dot(r_rc,r_qc)**2/r_rc_norm**2/r_qc_norm**2)),[0]*6), \
				  np.append(np.append(-((r_qc-r_bc)/r_bc_norm/r_qc_norm+(np.dot(r_bc,r_qc)*r_bc)/r_bc_norm**3/r_qc_norm+(np.dot(r_bc,r_qc)*r_qc)/r_bc_norm/r_qc_norm**3)/sqrt(1 - np.dot(r_bc,r_qc)**2/r_bc_norm**2/r_qc_norm**2),[0]*3),\
				  np.append(-(r_qc/r_bc_norm/r_qc_norm-(np.dot(r_bc,r_qc)*r_bc)/r_bc_norm**3/r_qc_norm)/sqrt(1 - np.dot(r_bc,r_qc)**2/r_bc_norm**2/r_qc_norm**2),[0]*3)), \
				  np.append(np.append(-((r_qr-r_cr)/r_cr_norm/r_qr_norm+(np.dot(r_cr,r_qr)*r_cr)/r_cr_norm**3/r_qr_norm+(np.dot(r_cr,r_qr)*r_qr)/r_cr_norm/r_qr_norm**3)/sqrt(1 - np.dot(r_cr,r_qr)**2/r_cr_norm**2/r_qr_norm**2), \
				  -(r_qr/r_cr_norm/r_qr_norm-(np.dot(r_cr,r_qr)*r_cr)/r_cr_norm**3/r_qr_norm)/sqrt(1 - np.dot(r_cr,r_qr)**2/r_cr_norm**2/r_qr_norm**2)),[0]*6), \
				  np.append(np.append([0]*3,-((r_qr-r_br)/r_br_norm/r_qr_norm+(np.dot(r_br,r_qr)*r_br)/r_br_norm**3/r_qr_norm+(np.dot(r_br,r_qr)*r_qr)/r_br_norm/r_qr_norm**3)/sqrt(1 - np.dot(r_br,r_qr)**2/r_br_norm**2/r_qr_norm**2)), \
				  np.append(-(r_qr/r_br_norm/r_qr_norm-(np.dot(r_br,r_qr)*r_br)/r_br_norm**3/r_qr_norm)/sqrt(1 - np.dot(r_br,r_qr)**2/r_br_norm**2/r_qr_norm**2),[0]*3)), \
				  np.append(np.append(-((r_qb-r_cb)/r_cb_norm/r_qb_norm+(np.dot(r_cb,r_qb)*r_cb)/r_cb_norm**3/r_qb_norm+(np.dot(r_cb,r_qb)*r_qb)/r_cb_norm/r_qb_norm**3)/sqrt(1 - np.dot(r_cb,r_qb)**2/r_cb_norm**2/r_qb_norm**2),[0]*3), \
				  np.append(-(r_qb/r_cb_norm/r_qb_norm-(np.dot(r_cb,r_qb)*r_cb)/r_cb_norm**3/r_qb_norm)/sqrt(1 - np.dot(r_cb,r_qb)**2/r_cb_norm**2/r_qb_norm**2),[0]*3)), \
				  np.append(np.append([0]*3,-((r_qb-r_rb)/r_rb_norm/r_qb_norm+(np.dot(r_rb,r_qb)*r_rb)/r_rb_norm**3/r_qb_norm+(np.dot(r_rb,r_qb)*r_qb)/r_rb_norm/r_qb_norm**3)/sqrt(1 - np.dot(r_rb,r_qb)**2/r_rb_norm**2/r_qb_norm**2)), \
				  np.append(-(r_qb/r_rb_norm/r_qb_norm-(np.dot(r_rb,r_qb)*r_rb)/r_rb_norm**3/r_qb_norm)/sqrt(1 - np.dot(r_rb,r_qb)**2/r_rb_norm**2/r_qb_norm**2),[0]*3)) \
				  ])

	b = np.array([gamma_safe_car*[r_qc_norm**2 - d_safe_car**2]-2*np.dot(-r_qc,Vq), \
				  gamma_safe_car*[r_qr_norm**2 - d_safe_car**2]-2*np.dot(-r_qr,Vq), \
				  gamma_safe_car*[r_qb_norm**2 - d_safe_car**2]-2*np.dot(-r_qb,Vq), \
				  gamma_measuring*[d_measuring**2 - r_qc_norm**2]+2*np.dot(-r_qc,Vq), \
				  gamma_measuring*[d_measuring**2 - r_qr_norm**2]+2*np.dot(-r_qr,Vq), \
				  gamma_measuring*[d_measuring**2 - r_qb_norm**2]+2*np.dot(-r_qb,Vq), \
				  (gamma_safe_uav/2)*[r_cr_norm**2 - d_safe_uav**2], \
				  (gamma_safe_uav/2)*[r_cb_norm**2 - d_safe_uav**2], \
				  (gamma_safe_uav/2)*[r_rc_norm**2 - d_safe_uav**2], \
				  (gamma_safe_uav/2)*[r_rb_norm**2 - d_safe_uav**2], \
				  (gamma_safe_uav/2)*[r_bc_norm**2 - d_safe_uav**2], \
				  (gamma_safe_uav/2)*[r_br_norm**2 - d_safe_uav**2], \
				  #gamma_fov*[theta_fov_fix - theta_qc]])
				  gamma_fov*[theta_fov_fix - theta_qc], \
				  gamma_occ*[theta_occ - theta_occ_cr] \
				  +(r_rc/r_rc_norm/r_qc_norm - np.dot(r_rc,r_qc)*r_rc/r_rc_norm/r_qc_norm**3)/(sqrt(1 - np.dot(r_rc,r_qc)**2/r_rc_norm**2/r_qc_norm**2)), \
				  gamma_occ*[theta_occ - theta_occ_cb] \
				  +(r_bc/r_bc_norm/r_qc_norm - np.dot(r_bc,r_qc)*r_bc/r_bc_norm/r_qc_norm**3)/(sqrt(1 - np.dot(r_bc,r_qc)**2/r_bc_norm**2/r_qc_norm**2)), \
				  gamma_occ*[theta_occ - theta_occ_rc] \
				  +(r_cr/r_cr_norm/r_qr_norm - np.dot(r_cr,r_qr)*r_cr/r_cr_norm/r_qr_norm**3)/(sqrt(1 - np.dot(r_cr,r_qr)**2/r_cr_norm**2/r_qr_norm**2)), \
				  gamma_occ*[theta_occ - theta_occ_rb] \
				  +(r_br/r_br_norm/r_qr_norm - np.dot(r_br,r_qr)*r_br/r_br_norm/r_qr_norm**3)/(sqrt(1 - np.dot(r_br,r_qr)**2/r_br_norm**2/r_qr_norm**2)), \
				  gamma_occ*[theta_occ - theta_occ_bc] \
				  +(r_cb/r_cb_norm/r_qb_norm - np.dot(r_cb,r_qb)*r_cb/r_cb_norm/r_qb_norm**3)/(sqrt(1 - np.dot(r_cb,r_qb)**2/r_cb_norm**2/r_qb_norm**2)), \
				  gamma_occ*[theta_occ - theta_occ_br] \
				  +(r_rb/r_rb_norm/r_qb_norm - np.dot(r_rb,r_qb)*r_rb/r_rb_norm/r_qb_norm**3)/(sqrt(1 - np.dot(r_rb,r_qb)**2/r_rb_norm**2/r_qb_norm**2)) \
				  ])

def	qpsolver():
	global camera_cmd,ranging_cmd,bearing_cmd
	
	cons = []
	
	for i in range (b.size):
		cons.append({'type': 'eq', 'fun': cons_maker(i)})
	for i in range (b.size):
		cons.append({'type': 'ineq', 'fun': cons_maker1(i)})
	
	ini = tuple(np.zeros(b.size + 12))
	bnds = ((-1.0, 1.0),)*9+((-3.0, 3.0),)*3+ ((0, np.inf),)*b.size
	
	optimal = minimize(object_fun, ini, method='SLSQP', bounds=bnds, constraints=cons,options={'maxiter':10}).x
	print(object_fun(optimal[:12]))

	camera_cmd.vel.x = optimal[0]
	camera_cmd.vel.y = optimal[1]
	camera_cmd.vel.z = optimal[2]
	ranging_cmd.vel.x = optimal[3]
	ranging_cmd.vel.y = optimal[4]
	ranging_cmd.vel.z = optimal[5]
	bearing_cmd.vel.x = optimal[6]
	bearing_cmd.vel.y = optimal[7]
	bearing_cmd.vel.z = optimal[8]
	camera_cmd.yawRate = optimal[9]
	ranging_cmd.yawRate = optimal[10]
	bearing_cmd.yawRate = optimal[11]

	
	
if __name__ == '__main__':
	try:
		rospy.init_node('controller')
		cf_vel_pub1 = rospy.Publisher('/crazyflie1/cmd_velocity_world', VelocityWorld, queue_size=1)
		cf_vel_pub2 = rospy.Publisher('/crazyflie2/cmd_velocity_world', VelocityWorld, queue_size=1)
		cf_vel_pub3 = rospy.Publisher('/crazyflie3/cmd_velocity_world', VelocityWorld, queue_size=1)
		tracking_distance_pub = rospy.Publisher('/tracking_distance', output_tracking_distance, queue_size=1)
		theta_FOV_pub = rospy.Publisher('/FOV', output_FOV, queue_size=1)
		inter_uavs_distance_pub = rospy.Publisher('/inter_uavs_distance', output_inter_uavs_distance, queue_size=1)
		theta_occ_pub = rospy.Publisher('/occlusion', output_occlusion, queue_size=1)
		rate = rospy.Rate(30)
		while not rospy.is_shutdown():			
			rospy.Subscriber("/uavs_pose", output_uavs_pose, uavs_pose_cb)
			rospy.Subscriber("/estimated_data", output_ukf, ukf_cb)
			rospy.Subscriber("/formation_controller", output_uavs_controller, uavs_controller_cb)		
			if uavs_pose_flag & uavs_controller_flag:
				CBF()
				qpsolver()
				if optimal_sensing_mode:
					cf_vel_pub1.publish(camera_cmd)
					cf_vel_pub2.publish(ranging_cmd)
					cf_vel_pub3.publish(bearing_cmd)
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
