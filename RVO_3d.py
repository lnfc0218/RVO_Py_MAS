from math import sqrt
import numpy

from math import cos, sin, atan2, asin, acos

from math import pi as PI

global suitable_V
global unsuitable_V
global RVO_BA_all

def distance_3d(pose1, pose2=[0,0,0]):
    """ compute Euclidean distance for 3D """
    # Maybe we need to specify a smaller offset to handle "divided by zero"?
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2+(pose1[2]-pose2[2])**2)+0.001

def in_between_3d(cone, dif):
    """compare angle between vectors to decide if it is within the cone region
    Inputs:
        cone:
            [azimuth, zenith, half_apex_angle]
            First two are spherical coordinate angles, while r>0;
            Third is the boundary we intend to compare vector angle with
        dif:
            Cartesian coordinates"""
    # convert Cartesian to spherical
    difSpherical = cartesian2spherical_unit(dif)
    interAngle = spherical_angle2intersection_angle(difSpherical,cone[0:2])
    if interAngle >= cone[2]:
        return False
    else:
        return True

def cartesian2spherical_unit(cart):
#    radius = distance_3d(cart, [0,0,0])
    radius = distance_3d(cart)
    azimuth = atan2(cart[1], cart[0])
    zenith = acos(cart[2]/radius)
    return [azimuth, zenith]

def spherical_angle2intersection_angle(ang0, ang1):
    azim0 = ang0[0]
    azim1 = ang1[0]
    zen0 = ang0[1]
    zen1 = ang1[1]
    azim_diff = azim0-azim1
# http://math.stackexchange.com/questions/231221/great-arc-distance-between-two-points-on-a-unit-sphere?newreg=a31d2b0763c54206a76c6af5c898ed42
    return acos(sin(zen0)*sin(zen1)*cos(azim_diff)+cos(zen0)*cos(zen1))# acos() ranges: [0,pi]


def RVO_update_3d(X, V_des, V_current, ws_model):
    """ compute best velocity given the desired velocity, current velocity and workspace model"""
    global RVO_BA_all
    ROB_RAD = ws_model['robot_radius']+0.01
    V_opt = list(V_current)
    for i in range(len(X)):
        vA = [V_current[i][0], V_current[i][1], V_current[i][2]]
        pA = [X[i][0], X[i][1], X[i][2]]
        RVO_BA_all = []
        for j in range(len(X)):
            if i!=j:
                vB = [V_current[j][0], V_current[j][1], V_current[j][2]]
                pB = [X[j][0], X[j][1], X[j][2]]
                dist_BA = distance_3d(pA, pB)
                # get azimuth and zenith angles for spherical cooridinates
                # will not use Greek letters because there are different notations in physics and mathematics
                azimuth = atan2(pB[1]-pA[1], pB[0]-pA[0]) # atan2 takes care of "divided by zero" case
                zenith = acos((pB[2]-pA[2])/dist_BA)
                if dist_BA < 2*ROB_RAD:# limit input of asin() to [-1,1] for intersect cases
                    dist_BA = 2*ROB_RAD
                half_apex_angle = asin(2*ROB_RAD/dist_BA)
                # parameters for a cone with apex at origin
                # azimuth and zenith specify axis, half_apex_angle
                cone_param = [azimuth, zenith, half_apex_angle]
                # use RVO
                transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1]), pA[2]+0.5*(vB[2]+vA[2])]
                RVO_BA = [transl_vB_vA, cone_param, dist_BA, 2*ROB_RAD]
                RVO_BA_all.append(RVO_BA)
        vA_post = intersect_3d(pA, V_des[i])
        V_opt[i] = vA_post[:]
    return V_opt

def judgeSuitable(new_v, pA, suit = True):
    global suitable_V
    global unsuitable_V
    for RVO_BA in RVO_BA_all:
        p_0 = RVO_BA[0]
        cone = RVO_BA[1]
        dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1], new_v[2]+pA[2]-p_0[2]]
        if in_between_3d(cone, dif):
            suit = False
            break
    if suit:
        suitable_V.append(new_v)
    else:
        unsuitable_V.append(new_v)

def intersect_3d(pA, vA):
    global suitable_V
    global unsuitable_V
    print '----------------------------------------'
#    print 'Start intersection test'
    norm_v = distance_3d(vA)
    suitable_V = []
    unsuitable_V = []
    # loops over all sampling points within the cone
    # the Bachelor thesis guy use projection to xy plane for 3D extension
    # messy angle comparison, you are welcome to improve it with quaternions!
    gear = 2 # Assume 5 level of gear here?
    step = 0.5 # radian step, can be bigger for faster computations
    azim_range = 2*PI*gear
    # TODO: change for loops into while loops so that we do not need to use NumPy
    for azim in numpy.arange(0, azim_range, step):# loops over azimuth
        for zen in numpy.arange(0, PI, step):
        # draw radian in spiral to save one for loop
            rad = 0.02 + azim / azim_range * norm_v # get radius, can be made into incrementation
            # find possible velocities? 
            new_v = [rad*cos(azim)*sin(zen), rad*sin(azim)*sin(zen), rad*cos(zen)]
            judgeSuitable(new_v, pA)
    new_v = vA[:]
    judgeSuitable(new_v, pA)
    #----------------------
    if suitable_V:
#        print 'Suitable found'
        vA_post = min(suitable_V, key = lambda v: distance_3d(v, vA))
    else:
        print 'Suitable not found'
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                cone = RVO_BA[1]
                dist = RVO_BA[2]
                rad = RVO_BA[3]
                dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1], unsuit_v[2]+pA[2]-p_0[2]]
                if in_between_3d(cone, dif):
                    difSpherical = cartesian2spherical_unit(dif)
                    interAngle = spherical_angle2intersection_angle(difSpherical,cone[0:2])
                    rad_ref = abs(dist * sin(interAngle))
                    if rad_ref >= rad:
                        rad = rad_ref
                    largerAngle = asin(rad_ref/rad)
                    dist_tg = abs(dist*cos(interAngle))-abs(rad*cos(largerAngle))
                    if dist_tg < 0:
                        dist_tg = 0
                    tc_v = dist_tg/distance_3d(dif)
                    tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc)+0.001
        WT = 0.2
        vA_post = min(unsuitable_V, key = lambda v: ((WT/tc_V[tuple(v)])+distance_3d(v, vA)))
    return vA_post


def compute_V_des_3d(X, goal, V_max):
    V_des = []
    for i in xrange(len(X)):
        dif_x = [goal[i][k]-X[i][k] for k in xrange(3)]
#        print 'dif_x='+str(dif_x)
        norm = distance_3d(dif_x)
        norm_dif_x = [dif_x[k]*V_max[k]/norm for k in xrange(3)]
#        print 'norm_dif_x='+str(norm_dif_x)
        V_des.append(norm_dif_x[:])
        if reach(X[i], goal[i], 0.01):# We want our mini drones to go really close to final destination right?
            V_des[i][0] = 0
            V_des[i][1] = 0
            V_des[i][2] = 0
    return V_des

def reach(p1, p2, bound=0.5):
    if distance_3d(p1,p2)< bound:
        return True
    else:
        return False


