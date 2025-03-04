
import yasmin
from yasmin import YASMIN_LOG_INFO, YASMIN_LOG_ERROR
yasmin.YASMIN_LOG_DEBUG = lambda text: None # Yasmin is very verbose and would flood the terminal
import time
from threading import Thread, Lock

from std_msgs.msg import Int16

from .an_const import ElevatorOrder, ElevatorCallback, ClampOrder, ClampCallback, WAIT_TIME

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

class HardwareOrder(yasmin.State):

    def __init__(self, logger, publisher, cb_key, order, pending, expected, timeout=WAIT_TIME):
        super().__init__(outcomes=['fail','success','preempted'])
        self._logger = logger
        self._publisher = publisher
        self._cb_key = cb_key
        self._order = order
        self._pending = pending
        self._expected = expected
        self._timeout = timeout
        self._msg = Int16()

    def execute(self, userdata):
        userdata[self._cb_key] = self._pending

        self._msg.data = self._order.value
        self._publisher.publish(self._msg)
   
        begin = time.perf_counter()
        while time.perf_counter() - begin < self._timeout:
            
            if self.is_canceled():
                return 'preempted'       

            response = userdata[self._cb_key]
            if response == self._expected:
                return 'success'
            elif response != self._pending:
                self._logger.warning(f"Unexpected response from hardware: {response} for order {self._order}")
                return 'fail'
            time.sleep(0.01)
        
        # timeout
        self._logger.warning(f"Timeout while waiting for response from hardware for order {self._order}")        
        return 'fail' 
   
class RiseElevator(HardwareOrder):
    
    def __init__(self, node, etage):
        super().__init__(node.get_logger(), node.elevator_pub, f"cb_elevator_{etage}", ElevatorOrder.MOVE_UP, ElevatorCallback.PENDING, ElevatorCallback.UP)
        self._debug_print = node.debug_print
        self.etage = etage
        
    def execute(self, userdata):        
        self._debug_print('c', f"Request to move elevator n°{self.etage} up")
        return super().execute(userdata)
 
class DescendElevator(HardwareOrder):
    
    def __init__(self, node, etage):
        super().__init__(node.get_logger(), node.elevator_pub, f"cb_elevator_{etage}", ElevatorOrder.MOVE_DOWN, ElevatorCallback.PENDING, ElevatorCallback.DOWN)
        self._debug_print = node.debug_print
        self.etage = etage
    
    def execute(self, userdata):        
        self._debug_print('c', f"Request to move elevator n°{self.etage} down")
        return super().execute(userdata)

class OpenClamp(HardwareOrder):
    
    def __init__(self, node, etage):
        super().__init__(node.get_logger(), node.clamp_pub, f"cb_clamp_{etage}", ClampOrder.OPEN, ClampCallback.PENDING, ClampCallback.OPEN)
        self._debug_print = node.debug_print
        self.etage = etage
    
    def execute(self, userdata):        
        self._debug_print('c', f"Request to open clamp n°{self.etage}")
        return super().execute(userdata)
       
class CloseClamp(HardwareOrder):
    
    def __init__(self, node, etage):
        super().__init__(node.get_logger(), node.clamp_pub, f"cb_clamp_{etage}", ClampOrder.CLOSE, ClampCallback.PENDING, ClampCallback.CLOSED)
        self._debug_print = node.debug_print
        self.etage = etage
    
    def execute(self, userdata):        
        self._debug_print('c', f"Request to close clamp n°{self.etage}")
        return super().execute(userdata)

class Sequence(yasmin.StateMachine):
    def __init__(self, outcomes = ["success", "fail", "preempted"], states = []):
        super().__init__(outcomes)
        self._last_state = None
        self._connector = None
        for state in states:
            self.add_state(state[0], state[1], connector=state[2] if len(state) > 2 else "success")

    def add_state(self, name, state, connector="success"):
        if self._last_state is not None:
            self.get_states()[self._last_state]["transitions"][self._connector] = name
        
        self._last_state = name
        self._connector = connector

        super().add_state(name, state, transitions={outcome: outcome for outcome in self.get_outcomes() if outcome in state.get_outcomes()})

    def set_start_state(self, name):
        if self.get_start_state():
            raise NotImplementedError("Cannot set start state on Sequence")
        else:
            super().set_start_state(name)

class Concurrence(yasmin.StateMachine):
    def __init__(self, /, **states):
        super().__init__(outcomes=["success", "fail", "preempted"])
        self._threads = {}
        self._lock = Lock()
        self._results = {}

        for name, state in states.items():
            self.add_state(name, state)

    def add_state(self, name, state, /):
        super().add_state(name, state, transitions={outcome: outcome for outcome in self.get_outcomes()})

    def execute(self, blackboard):
        with self._lock:
            if self._threads != {}:
                raise RuntimeError("Concurrence is already running")
            self._results.clear()
            for state in self.get_states():
                if self.is_canceled(): break
                self._threads[state] = Thread(target=self._run_state, args=(state, blackboard))
                self._threads[state].start()

        while True:
            with self._lock:
                if len(self._results) == len(self._threads):
                    break
            time.sleep(0.1)

        self._threads.clear()

        if any(result == "preempted" for result in self._results.values()):
            return "preempted"
        if all(result == "success" for result in self._results.values()):
            return "success"
        
        return "fail"

    def cancel_state(self):
        super().cancel_state()
        with self._lock:
            for state in self._threads:
                self.get_states()[state]["state"].cancel_state()
        YASMIN_LOG_INFO("Cancelling concurrent states...")

    def set_start_state(self, name):
        if self.get_start_state():
            raise NotImplementedError("Cannot set start state on Concurrence")
        else:
            super().set_start_state(name)

    def _run_state(self, state, blackboard):
        YASMIN_LOG_INFO(f"Starting concurrent state {state}")
        try:
            outcome = self.get_states()[state]["state"](blackboard)
            YASMIN_LOG_INFO(f"Concurrent state {state} returned outcome {outcome}")
        except Exception as e:
            outcome = "fail"
            YASMIN_LOG_ERROR(f"State {state} raised exception: {repr(e)}")

        with self._lock:
            self._results[state] = outcome

        if outcome in ["fail", "preempted"]:
            self.cancel_state()