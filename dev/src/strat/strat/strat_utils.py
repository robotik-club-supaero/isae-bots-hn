from geometry_msgs.msg import Quaternion
from message.msg import EndOfActionMsg

def adapt_pos_to_color(x, y, theta, color):
    if color == 0:
        return x, y, theta
    if color == 1:
        return x, 2000 - y, theta if theta in (0, 3.14) else -theta
    else:
        print("\nERROR in 'adapt_pos_to_color' !")
        return x, y, theta
    
def create_quaternion(x, y, z, w):
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))

def create_end_of_action_msg(exit, reason):
    msg = EndOfActionMsg()
    msg.exit = exit
    msg.reason = reason

    return msg
