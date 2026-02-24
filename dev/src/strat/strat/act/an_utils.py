import yasmin # type: ignore
from yasmin import YASMIN_LOG_INFO, YASMIN_LOG_ERROR # type: ignore # State machine manager library
yasmin.YASMIN_LOG_DEBUG = lambda text: None # Yasmin is very verbose and would flood the terminal

import time
from threading import Thread, Lock

from std_msgs.msg import Int16 # type: ignore

from .an_const import DrawbridgeOrder, DrawbridgeCallback, CursorOrder, CursorCallback, PumpsOrder, PumpsCallback, WAIT_TIME

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

class HardwareOrder(yasmin.State): # Should not be modified (unless you should)

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
            
            if self.is_canceled(): return 'preempted'  

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

# -------- Order to be sent to the Action BN specific of the year ---------- #

class DrawbridgeUP(HardwareOrder):
    
    def __init__(self, node):
        super().__init__(node.get_logger(), node.drawbridge_pub, f"cb_drawbridge", DrawbridgeOrder.UP, DrawbridgeCallback.PENDING, DrawbridgeCallback.UP)
        self._debug_print = node.debug_print
        
    def execute(self, userdata):        
        self._debug_print('c', f"Request to set DRAWBRIDGE in UP position..")
        return super().execute(userdata)

class DrawbridgeDOWN(HardwareOrder):
    
    def __init__(self, node):
        super().__init__(node.get_logger(), node.drawbridge_pub, f"cb_drawbridge", DrawbridgeOrder.DOWN, DrawbridgeCallback.PENDING, DrawbridgeCallback.DOWN)
        self._debug_print = node.debug_print
        
    def execute(self, userdata):        
        self._debug_print('c', f"Request to set DRAWBRIDGE in DOWN position..")
        return super().execute(userdata)

class PumpsON(HardwareOrder):
    
    def __init__(self, node):
        super().__init__(node.get_logger(), node.pumps_pub, f"cb_pumps", PumpsOrder.ON, PumpsCallback.PENDING, PumpsCallback.ON)
        self._debug_print = node.debug_print
        
    def execute(self, userdata):        
        self._debug_print('c', f"Request to set PUMPS in ON position..")
        return super().execute(userdata)

class PumpsOFF(HardwareOrder):
    
    def __init__(self, node):
        super().__init__(node.get_logger(), node.pumps_pub, f"cb_pumps", PumpsOrder.OFF, PumpsCallback.PENDING, PumpsCallback.OFF)
        self._debug_print = node.debug_print
        
    def execute(self, userdata):        
        self._debug_print('c', f"Request to set PUMPS in OFF position..")
        return super().execute(userdata)


class CursorStickDOWN(HardwareOrder):
    
    def __init__(self, node):
        super().__init__(node.get_logger(), node.cursor_stick_pub, f"cb_cursor_stick", CursorOrder.DOWN, PumpsCallback.PENDING, CursorCallback.DOWN)
        self._debug_print = node.debug_print
        
    def execute(self, userdata):        
        self._debug_print('c', f"Request to set CURSOR_STICK in DOWN position..")
        return super().execute(userdata)

class CursorStickUP(HardwareOrder):
    
    def __init__(self, node):
        super().__init__(node.get_logger(), node.cursor_stick_pub, f"cb_cursor_stick", CursorOrder.UP, PumpsCallback.PENDING, CursorCallback.UP)
        self._debug_print = node.debug_print
        
    def execute(self, userdata):        
        self._debug_print('c', f"Request to set CURSOR_STICK in UP position..")
        return super().execute(userdata)

# ------------------------------------------------------------------ #

# To make a sequence of action (used in the states machines)
class Sequence(yasmin.StateMachine): # Should not be modified (unless you should)
    def __init__(self, outcomes = ["success", "fail", "preempted"], states = [], success_outcome="success"):
        super().__init__(outcomes)
        self._last_state = None
        self._connector = None
        self._success_outcome = success_outcome

        for state in states:
            self.add_state(state[0], state[1], connector=state[2] if len(state) > 2 else "success")
        
    def add_state(self, name, state, connector="success"):
        if self._last_state is not None:
            self.get_states()[self._last_state]["transitions"][self._connector] = name
        
        self._last_state = name
        self._connector = connector

        super().add_state(name, state, transitions={outcome: outcome for outcome in self.get_outcomes() if outcome in ( ["preempted", "success", "fail"] + list(state.get_outcomes()))})

    def cancel_state(self):
        super().cancel_state()

    def execute(self, blackboard):
        try:
            outcome = super().execute(blackboard)
            if self.is_canceled(): return "preempted"
            if outcome == self._connector:
                return self._success_outcome
            else:
                return outcome
        except RuntimeError:
            if self.is_canceled(): return "preempted"
            else:
                raise

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