from geometry_msgs.msg import Quaternion
from message.msg import EndOfActionMsg

def adapt_pos_to_side(x, y, theta, color):
    if color == 0:
        return x, y, theta
    else:
        return x, 3000-y, -theta        # si la symétrie est selon l'axe y
        # return 2000-x, y, theta + pi  # si la symétrie est selon l'axe x
     
def create_quaternion(x, y, z, w):
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))

def create_end_of_action_msg(exit, reason):
    msg = EndOfActionMsg()
    msg.exit = exit
    msg.reason = reason

    return msg
