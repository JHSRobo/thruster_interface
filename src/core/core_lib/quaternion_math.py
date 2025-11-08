# This is one of ROVOTICS' internal libraries.
# Used for handling quaternion status and conversion from vectors.

from geometry_msgs.msg import Quaternion
from math import sin, cos

# Takes in roll, pitch, yaw and spits out a quaternion orientation
def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

# Transforms the first quaternion with the second.
# Mathematically, this is done by multiplying them.
def quaternion_multiply(Q0,Q1):
    fq = Quaternion()

    # Computer the product of the two quaternions, term by term
    fq.w = Q0.w * Q1.w - Q0.x * Q1.x - Q0.y * Q1.y - Q0.z * Q1.z
    fq.x = Q0.w * Q1.x + Q0.x * Q1.w + Q0.y * Q1.z - Q0.z * Q1.y
    fq.y = Q0.w * Q1.y - Q0.x * Q1.z + Q0.y * Q1.w + Q0.z * Q1.x
    fq.z = Q0.w * Q1.z + Q0.x * Q1.y - Q0.y * Q1.x + Q0.z * Q1.w

    return fq