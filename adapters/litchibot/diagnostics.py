"""Opt-in observation only. Records SDK frames without changing their values."""
from dataclasses import asdict, is_dataclass
from enum import Enum
import json
from pathlib import Path
import threading
import time

import numpy as np


def json_safe(value):
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if is_dataclass(value):
        return json_safe(asdict(value))
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value,float) and not np.isfinite(value):
        return None
    return value


class DiagnosticLog:
    def __init__(self,path):
        self.lock = threading.Lock()
        self.stream = Path(path).open('x',encoding='utf-8')
        self.write('clock_anchor',{'unix_ns':time.time_ns(),'monotonic_ns':time.monotonic_ns()})

    def write(self,kind,data):
        with self.lock:
            if self.stream is not None:
                self.stream.write(json.dumps({'type':kind,'logged_monotonic_ns':time.monotonic_ns(),
                    'data':json_safe(data)},allow_nan=False)+'\n')

    def raw(self,frame):
        self.write('raw',frame.to_dict())

    def solved(self,frame):
        self.write('solved',frame.to_dict())

    def close(self):
        with self.lock:
            if self.stream is not None:
                self.stream.close()
                self.stream = None
