from message.msg import EndOfActionMsg

def create_end_of_action_msg(exit, reason):
    msg = EndOfActionMsg()
    msg.exit = exit
    msg.reason = reason

    return msg
