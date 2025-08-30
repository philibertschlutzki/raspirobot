# filters.py
from collections import deque

def median(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0: return None
    return s[n//2] if n % 2 else 0.5*(s[n//2-1] + s[n//2])

class FilteredDistance:
    def __init__(self, window=5):
        self.buf = deque(maxlen=window)
    def push(self, v):
        if v is not None:
            self.buf.append(v)
    def value(self):
        return median(list(self.buf))
