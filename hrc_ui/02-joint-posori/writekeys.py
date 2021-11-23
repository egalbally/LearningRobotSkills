import redis 
import sys
import numpy as np
import time
import json

def rot_x(theta):
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    return np.array([[1, 0, 0], [0, c_theta, -s_theta], [0, s_theta, c_theta]])

def rot_y(theta):
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    return np.array([[c_theta, 0, s_theta], [0, 1, 0], [-s_theta, 0, c_theta]])

def rot_z(theta):
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    return np.array([[c_theta, -s_theta, 0], [s_theta, c_theta, 0], [0, 0, 1]])
    
def zyx_euler_angles_to_mat(alpha, beta, gamma):
    return rot_z(alpha) @ rot_y(beta) @ rot_x(gamma)

if len(sys.argv) != 2:
    print('usage: python3 {} <# of joints>'.format(sys.argv[0]))
    exit(0)

try:
    num_joints = int(sys.argv[1])
except:
    print('usage: python3 {} <# of joints>'.format(sys.argv[0]))
    exit(0)

# mode key
MODE_KEY = 'sai2::interfaces::tutorial::mode'

# joint task keys
JOINT_KEY = 'sai2::interfaces::tutorial::q'
KP_GAIN_KEY = 'sai2::interfaces::tutorial::joint_kp'
KV_GAIN_KEY = 'sai2::interfaces::tutorial::joint_kv'

# posori task keys
EE_POS_KEY = 'sai2::interfaces::tutorial::ee_pos'
EE_ORI_KEY = 'sai2::interfaces::tutorial::ee_ori'
EE_POS_KP_KEY = 'sai2::interfaces::tutorial::ee_pos_kp'
EE_POS_KV_KEY = 'sai2::interfaces::tutorial::ee_pos_kv'
EE_ORI_KP_KEY = 'sai2::interfaces::tutorial::ee_ori_kp'
EE_ORI_KV_KEY = 'sai2::interfaces::tutorial::ee_ori_kv'
EE_ROTMAT_KEY = 'sai2::interfaces::tutorial::ee_rotmat'

initial_joint_values = 2 * np.pi * np.random.rand(num_joints)
initial_pos_values = 3 * np.random.rand(3)
initial_ori_values = 2 * np.pi * np.random.rand(3)

r = redis.Redis()

# mode
r.set(MODE_KEY, 'joint')

# joint task
r.set(JOINT_KEY, str(initial_joint_values.tolist()))
r.set(KP_GAIN_KEY, '100')
r.set(KV_GAIN_KEY, '2')

# posori task
r.set(EE_POS_KEY, str(initial_pos_values.tolist()))
r.set(EE_ORI_KEY, str(initial_ori_values.tolist()))
r.set(EE_POS_KP_KEY, '100')
r.set(EE_POS_KV_KEY, '2')
r.set(EE_ORI_KP_KEY, '75')
r.set(EE_ORI_KV_KEY, '3')

# print joint
print('{} set to {}'.format(JOINT_KEY, r.get(JOINT_KEY)))
print('{} set to {}'.format(KP_GAIN_KEY, r.get(KP_GAIN_KEY)))
print('{} set to {}'.format(KV_GAIN_KEY, r.get(KV_GAIN_KEY)))

# print posori
print('{} set to {}'.format(EE_POS_KEY, r.get(EE_POS_KEY)))
print('{} set to {}'.format(EE_ORI_KEY, r.get(EE_ORI_KEY)))
print('{} set to {}'.format(EE_POS_KP_KEY, r.get(EE_POS_KP_KEY)))
print('{} set to {}'.format(EE_POS_KV_KEY, r.get(EE_POS_KV_KEY)))
print('{} set to {}'.format(EE_ORI_KP_KEY, r.get(EE_ORI_KP_KEY)))
print('{} set to {}'.format(EE_ORI_KV_KEY, r.get(EE_ORI_KV_KEY)))

while True:
    raw_ori = r.get(EE_ORI_KEY)
    gamma, beta, alpha = json.loads(raw_ori)
    rmat = zyx_euler_angles_to_mat(alpha, beta, gamma)
    r.set(EE_ROTMAT_KEY, str(rmat.tolist()))
    time.sleep(0.5)
