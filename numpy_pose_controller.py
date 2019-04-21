import numpy as np

def DetermineControllerSpeeds(pose_goal, pose_current):
    kp_x, kp_y, kp_theta = 0.4, 0.4, 0.1
    threshold_distance = 0.55
    threshold_angle = np.pi/10
    
    #pose_goal = np.array([[5.0], [4.0], [0.0]], np.float32)
    #pose_current = np.array([[5.0], [4.1], [np.pi/2]], np.float32)

    # Obtain delta of goal and current poses (direction vector in the global frame)
    delta_pose = pose_goal - pose_current    
    #print(delta_pose)
    
    # Calculate Euclidian distance from quadcopter to goal
    linear_distance =  np.sqrt(np.square(delta_pose[0][0]) + np.square(delta_pose[1][0]))
    #print(linear_distance)

    # Check if quadcopter is near the goal
    if linear_distance <= threshold_distance:
        diff_theta = delta_pose[2][0]
        
        # Check if the quadcopter is almost oriented as the goal to stop moving it
        if abs(diff_theta) <= threshold_angle:
            return [0.0, 0.0, 0.0, True]
        
        # Correct orientation by considering rotation speed only 
        v_theta = kp_theta * delta_pose[2][0]
        if abs(v_theta) > 1.0:
            v_theta /= abs(v_theta) # Transform theta speed to 1 or -1 if it is too big
        return [0.0, 0.0, v_theta, False]

    # Check current theta of quadcopter
    theta_current = pose_current[2][0]
    
    # Multiply the difference of poses times the global-to-local rotation matrix 
    rotation_matrix = np.array([[np.cos(theta_current), np.sin(theta_current), 0],[-np.sin(theta_current), np.cos(theta_current), 0],[0, 0, 1]], np.float32)
    delta_pose_local = np.matmul(rotation_matrix, delta_pose)

    # Calculate necessary x speed
    v_x = kp_x * delta_pose_local[0][0]
    if abs(v_x) > 1.0:
        v_x /= abs(v_x) # Transform x speed to 1 or -1 if it is too big

    # Calculate necessary y speed
    v_y = kp_y * delta_pose_local[1][0]
    if abs(v_y) > 1.0:
        v_y /= abs(v_y) # Transform y speed to 1 or -1 if it is too big

    #print(delta_pose_local)
    #print(rotation_matrix)
    return [v_x, v_y, 0.0, False]
