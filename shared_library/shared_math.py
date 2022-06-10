import math
import numpy as np
from shapely.geometry import Polygon
from shapely.geometry import box, Point
from shapely.affinity import rotate, translate


''' Helper function to calculate the difference between 2 angles in radians '''
def angleDifference( angle1, angle2 ):
    diff = ( angle2 - angle1 + math.pi ) % (2*math.pi) - math.pi
    if diff < -math.pi:
        return diff + (2*math.pi)
    else:
        return diff

''' Helper function to calculate if a target is in range and fov of a sensor '''
def check_in_range_and_fov(target_angle, distance, center_angle, horizontal_fov, max_range):
    angle_diff = ((center_angle - target_angle + math.pi + (2*math.pi)) % (2*math.pi)) - math.pi
    if abs(angle_diff) <= (horizontal_fov/2.0) and (distance <= max_range):
        return True
    return False

''' Helper function to calculate if a target is in fov of a sensor '''
def check_in_fov(target_angle, center_angle, horizontal_fov):
    angle_diff = ((center_angle - target_angle + math.pi + (2*math.pi)) % (2*math.pi)) - math.pi
    if abs(angle_diff) <= (horizontal_fov/2.0):
        return True
    return False

''' Calculate the distance and angle parameters of an observation '''
def get_relative_detection_params(detector_x, detector_y, detector_theta, observation_x, observation_y):
    # Get the relative coordinates
    delta_x = observation_x - detector_x
    delta_y = observation_y - detector_y
    # Get the target anlge
    target_line_angle = math.atan2(delta_y, delta_x)
    # Calc relative angle from vehicle angle
    relative_angle_to_detector = angleDifference(target_line_angle, detector_theta)
    # Calc distance
    relative_distance = math.hypot(delta_x, delta_y)
    return relative_angle_to_detector, target_line_angle, relative_distance

''' Kalman filter prediction equasion '''
def kalman_prediction(X_hat_t_1, P_t_1, F_t, B_t, U_t, Q_t):
    X_hat_t = F_t.dot(X_hat_t_1) + (B_t.dot(U_t).reshape(B_t.shape[0], -1))
    P_t = F_t.dot(P_t_1).dot(F_t.transpose()) + Q_t
    return X_hat_t, P_t

''' Kalman filter update equasion '''
def kalman_update(X_hat_t, P_t, Z_t, R_t, H_t):
    K_prime = P_t.dot(H_t.transpose()).dot(kalman_inverse(H_t.dot(P_t).dot(H_t.transpose()) + R_t))
    X_t = X_hat_t + K_prime.dot(Z_t - H_t.dot(X_hat_t))
    P_t = P_t - K_prime.dot(H_t).dot(P_t)
    return X_t, P_t

''' Inverse function that is better than the default numpy one '''
def kalman_inverse(m):
    a, b = m.shape
    if a != b:
        raise ValueError("Only square matrices are invertible.")
    i = np.eye(a, a)
    return np.linalg.lstsq(m, i, rcond=None)[0]

def ellipsify(covariance, num_std_deviations = 3.0):
    # Eigenvalue and eigenvector computations
    #print ( covariance )
    vals, vecs = np.linalg.eigh(covariance)
    order = vals.argsort()[::-1]
    vals = vals[order]
    vecs = vecs[:, order]

    # Use the eigenvalue to figure out which direction is larger
    phi = np.arctan2(*vecs[:, 0][::-1])

    # A and B are radii
    #a, b = num_std_deviations * np.sqrt(vals)

    if not np.any(np.isnan(vals)) and np.all(np.isfinite(vals)):
        a, b = num_std_deviations * np.sqrt(vals)
    elif not np.isnan(vals[0]) and np.isfinite(vals[0]):
        a = num_std_deviations * np.sqrt(vals[0])
        b = 0.0
    elif not np.isnan(vals[1]) and np.isfinite(vals[1]):
        b = num_std_deviations * np.sqrt(vals[1])
        a = 0.0
    else:
        a = 0.0
        b = 0.0

    return a, b, phi

def calculateRadiusAtAngle(a, b, phi, measurementAngle):
    denominator = math.sqrt( a**2 * math.sin(phi-measurementAngle)**2 + b**2 * math.cos(phi-measurementAngle)**2 )
    if denominator == 0.0:
        #print ( "Warning: calculateEllipseRadius denom 0! - check localizer definitions " )
        return 0.0
    else:
        return ( a * b ) / denominator

# This function turns elipses into rectanges so that an IO calculation can be done for 
# ball tree matching
def computeDistanceEllipseBox(a, b):
    cx = a[0]
    cy = a[1]
    w = a[2]
    h = a[3]
    angle = a[4]
    c = box(-w/2.0, -h/2.0, w/2.0, h/2.0)
    rc = rotate(c, angle)
    contour_a = translate(rc, cx, cy)

    cx = b[0]
    cy = b[1]
    w = b[2]
    h = b[3]
    angle = a[4]
    c = box(-w/2.0, -h/2.0, w/2.0, h/2.0)
    rc = rotate(c, angle)
    contour_b = translate(rc, cx, cy)

    iou = contour_a.intersection(contour_b).area / contour_a.union(contour_b).area

    # Modify to invert the IOU so that it works with the BallTree class
    if iou <= 0:
        distance = 1
    else:
        distance = 1 - iou

    return distance

# This function turns elipses into rectanges so that an IO calculation can be done for 
# ball tree matching
def computeDistanceEuclidean(a, b):
    # cx = a[0]
    # cy = a[1]
    # w = a[2]
    # h = a[3]
    # angle = a[4]
    # c = box(-w/2.0, -h/2.0, w/2.0, h/2.0)
    # rc = rotate(c, angle)
    # contour_a = translate(rc, cx, cy)

    cx = b[0]
    cy = b[1]
    w = b[2]
    h = b[3]
    angle = a[4]
    c = box(-w/2.0, -h/2.0, w/2.0, h/2.0)
    rc = rotate(c, angle)
    contour_b = translate(rc, cx, cy)

    # If the boxes intersect, then we are within the gate of both
    if contour_b.contains(Point(a[0], a[1])):
        distance = math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        return distance / 100.0
    else:
        return 1

def RMSE(differences_list):
    mean_of_differences_squared = np.square(np.array(differences_list)).mean()
    rmse_val = np.sqrt(mean_of_differences_squared)
    return rmse_val