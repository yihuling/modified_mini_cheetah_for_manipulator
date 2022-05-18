import numpy as np
stand_up_height=0.05
l1=0.21
l2=0.20

h  = stand_up_height;
end_theta1 = -np.arccos((l1*l1+h*h-l2*l2)/(2*l1*h))
end_theta2 = (np.pi - np.arccos((l1*l1+l2*l2-h*h)/(2*l1*l2)))
print(end_theta1,end_theta2)
print(end_theta1*180/np.pi,end_theta2*180/np.pi)
