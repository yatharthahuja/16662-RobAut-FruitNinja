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

	# fa.goto_joints([0.0, -0.7, 0.0, -2.15, 0.0, 1.57, 0.7])

	# print("check 2")

	des_pose = RigidTransform(rotation = np.array([[0.999989403, 6.35691360e-04, 1.23997835e-03],
												[6.37934120e-04, -9.99988533e-01, -1.80916991e-03],
												[1.23881406e-03, 1.80994177e-03, -9.99997595e-01]]),
												translation = np.array([0.2, 0.2, 0.2]),
												from_frame="franka_tool",
												to_frame="world")
	
	delta = np.linalg.norm(fa.get_pose().translation - des_pose.translation)
	print(delta*2.5, int(delta*2.5), round(delta*2.5, 1))
	duration = round(delta*2.5, 1)
	print(float(duration), type(float(duration)), type(duration))
	duration = min(1, float(duration))
	fa.goto_pose(des_pose, use_impedance=False, duration=float(duration))

	print("check 2")

	des_pose = RigidTransform(rotation = np.array([[0.999989403, 6.35691360e-04, 1.23997835e-03],
												[6.37934120e-04, -9.99988533e-01, -1.80916991e-03],
												[1.23881406e-03, 1.80994177e-03, -9.99997595e-01]]),
												translation = np.array([0.25, 0.25, 0.2]),
												from_frame="franka_tool",
												to_frame="world")
	
	print("check 3")
	
	delta = np.linalg.norm(fa.get_pose().translation - des_pose.translation)
	print(delta*2.5, int(delta*2.5), round(delta*2.5, 1))
	print(float(duration), type(float(duration)), type(duration))
	duration = min(1, float(duration))
	fa.goto_pose(des_pose, use_impedance=False, duration=float(duration))

	des_pose = RigidTransform(rotation = np.array([[0.999989403, 6.35691360e-04, 1.23997835e-03],
												[6.37934120e-04, -9.99988533e-01, -1.80916991e-03],
												[1.23881406e-03, 1.80994177e-03, -9.99997595e-01]]),
												translation = np.array([-0.25, 0.25, 0.2]),
												from_frame="franka_tool",
												to_frame="world")
	
	# print("check 3")

	# fa.goto_pose(des_pose, use_impedance=False)

	# des_pose = RigidTransform(rotation = np.array([[0.999989403, 6.35691360e-04, 1.23997835e-03],
	# 											[6.37934120e-04, -9.99988533e-01, -1.80916991e-03],
	# 											[1.23881406e-03, 1.80994177e-03, -9.99997595e-01]]),
	# 											translation = np.array([0.35, 0.1, 0.2]),
	# 											from_frame="franka_tool",
	# 											to_frame="world")
	
	# print("check 3")

	# fa.goto_pose(des_pose, use_impedance=False)

	# des_pose = RigidTransform(rotation = np.array([[0.999989403, 6.35691360e-04, 1.23997835e-03],
	# 											[6.37934120e-04, -9.99988533e-01, -1.80916991e-03],
	# 											[1.23881406e-03, 1.80994177e-03, -9.99997595e-01]]),
	# 											translation = np.array([0.35, -0.25, 0.2]),
	# 											from_frame="franka_tool",
	# 											to_frame="world")

	# fa.goto_pose(des_pose, use_impedance=False)

	fa.reset_joints()

	print("Done!")
