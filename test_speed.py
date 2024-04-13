import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm


height_z = 0.2 # plane const

if __name__ == "__main__":
	# fa = FrankaArm()
	fa = FrankaArm(with_gripper=False)

	fa.reset_joints()	

	T_ee_world = fa.get_pose()
	print("check 0")
	joints = fa.get_joints()

	# print("check 0")
	# gripper_width = fa.get_gripper_width()
	# print("check 0")
	# froce_torque = fa.get_ee_force_torque()

	print("check 1")

	des_pose = RigidTransform(rotation = np.array([[0.999989403, 6.35691360e-04, 1.23997835e-03],
												[6.37934120e-04, -9.99988533e-01, -1.80916991e-03],
												[1.23881406e-03, 1.80994177e-03, -9.99997595e-01]]),
												# translation = np.array([0.2, 0.2, 0.2]),
												translation = np.array([0.3, 0.1, 0.3]),
												from_frame="franka_tool",
												to_frame="world")
	

	fa.goto_pose(des_pose, use_impedance=False, duration=0.5)

	fa.reset_joints()