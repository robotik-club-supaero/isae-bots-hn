
import smach
import time

from an_const import DoorOrder, DoorCallback, ElevatorOrder, ElevatorCallback, ClampOrder, ClampCallback, WAIT_TIME

#################################################################
# Colors gestion												#
#################################################################

class Color():
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    RESET = '\033[0m'

color_dict = {'n':Color.BLACK, 'r':Color.RED, 'g':Color.GREEN, 'y':Color.YELLOW, 'b':Color.BLUE, 
             'm':Color.MAGENTA, 'c':Color.CYAN, 'w':Color.WHITE}

class HardwareOrder(smach.State):

    def __init__(self, logger, publisher, cb_key, order, pending, expected, timeout=WAIT_TIME):
        super().__init__(input_keys=[cb_key], output_keys=[cb_key], outcomes=['fail','success','preempted'])
        self._logger = logger
        self._publisher = publisher
        self._cb_key = cb_key
        self._order = order
        self._pending = pending
        self._expected = expected
        self._timeout = timeout

    def execute(self, userdata):
        getattr(userdata, self._cb_key)[0] = self._pending

        self._publisher.publish(self._order.value)
   
        begin = time.perf_counter()
        while time.perf_counter() - begin < self._timeout:
            
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'       

            response = getattr(userdata, self._cb_key)[0]
            if response == self._expected:
                return 'success'
            elif response != self._pending:
                self._logger.warning(f"Unexpected response from hardware: {response} for order {self._order}")
                return 'fail'
            time.sleep(0.01)
        
        # timeout
        self._logger.warning(f"Timeout while waiting for response from hardware for order {self._order}")        
        return 'fail'
     
class OpenDoors(HardwareOrder):
    
    def __init__(self, node):
        super().__init__(node.doors_pub, 'cb_doors', DoorOrder.OPEN, DoorCallback.PENDING, DoorCallback.OPEN)
        self._debug_print = node.debug_print
    
    def execute(self, userdata):
        self._debug_print('c', "Request to open doors")
        return super().execute(userdata)
    
class CloseDoors(HardwareOrder):
    
    def __init__(self, node):
        super().__init__(node.doors_pub, 'cb_doors', DoorOrder.CLOSE, DoorCallback.PENDING, DoorCallback.CLOSED)
        self._debug_print = node.debug_print

    def execute(self, userdata):
        self._debug_print('c', "Request to close doors")
        return super().execute(userdata)    
   
class RiseElevator(HardwareOrder):
    
    def __init__(self, node):
        super().__init__(node.elevator_pub, 'cb_elevator', ElevatorOrder.MOVE_UP, ElevatorCallback.PENDING, ElevatorCallback.UP)
        self._debug_print = node.debug_print
        
    def execute(self, userdata):        
        self._debug_print('c', "Request to move elevator up")
        return super().execute(userdata)
 
class DescendElevator(HardwareOrder):
    
    def __init__(self, node):
        super().__init__(node.elevator_pub, 'cb_elevator', ElevatorOrder.MOVE_DOWN, ElevatorCallback.PENDING, ElevatorCallback.DOWN)
        self._debug_print = node.debug_print
        
    def execute(self, userdata):        
        self._debug_print('c', "Request to move elevator down")
        return super().execute(userdata)

class OpenClamp(HardwareOrder):
    
    def __init__(self, node):
        super().__init__(node.clamp_pub, 'cb_clamp', ClampOrder.OPEN, ClampCallback.PENDING, ClampCallback.OPEN)
        self._debug_print = node.debug_print
        
    def execute(self, userdata):        
        self._debug_print('c', "Request to open clamp")
        return super().execute(userdata)
       
class CloseClamp(HardwareOrder):
    
    def __init__(self, node):
        super().__init__(node.clamp_pub, 'cb_clamp', ClampOrder.CLOSE, ClampCallback.PENDING, ClampCallback.CLOSED)
        self._debug_print = node.debug_print
        
    def execute(self, userdata):        
        self._debug_print('c', "Request to close clamp")
        return super().execute(userdata)

def _auto_keys(actions):
    input_keys = set()
    output_keys = set()
    for _, sm in actions:
        input_keys.update(sm.get_registered_input_keys())
        output_keys.update(sm.get_registered_output_keys())
    
    return list(input_keys), list(output_keys)
       
class AutoSequence(smach.Sequence):

    def __init__(self, *actions):
        input_keys, output_keys = _auto_keys(actions)     
        super().__init__( 
            input_keys = input_keys,
            output_keys = output_keys,
            outcomes = ['success', 'fail', 'preempted'],
            connector_outcome = 'success'
        )

        with self:
            for id, sm in actions:
                smach.Sequence.add(id, sm)

class AutoConcurrence(smach.Concurrence):

    def __init__(self, **actions):
        input_keys, output_keys = _auto_keys(actions.items())
        super().__init__( 
            input_keys = input_keys,
            output_keys = output_keys,
            outcomes = ['success', 'fail', 'preempted'],
            default_outcome = 'fail',
            child_termination_cb = AutoConcurrence._child_termination_cb,
            outcome_cb = AutoConcurrence._outcome_cb
        )

        with self:
            for id, sm in actions.items():
                smach.Concurrence.add(id, sm)

    @staticmethod
    def _child_termination_cb(outcomes):
        return any(outcome == 'preempted' for outcome in outcomes)

    @staticmethod
    def _outcome_cb(outcomes):
        if any(outcome == 'preempted' for outcome in outcomes.values()):
            return 'preempted'
        elif all(outcome == 'success' for outcome in outcomes.values()):
            return 'success'
        else:
            return 'fail'
