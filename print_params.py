import rospy

params_names = rospy.get_param_names()

for name in params_names:
	print(f'{name}: {rospy.get_param(name)}')
