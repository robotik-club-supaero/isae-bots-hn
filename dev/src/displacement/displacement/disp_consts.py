from enum import IntEnum
from strat.act.an_const import DspCallback

class BR_Callback(IntEnum):
    OK_ORDER = 7
    ERROR_ASSERV = 0
    # Non exhaustive (see source code of BR)
