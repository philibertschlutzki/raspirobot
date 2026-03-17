import sys
import math
import random
from unittest.mock import MagicMock

class MockNP:
    @staticmethod
    def array(lst, dtype=None):
        class NumpyArrayMock(list):
            @property
            def shape(self):
                return (len(self), 2)
            def __getitem__(self, idx):
                if isinstance(idx, tuple) and len(idx) == 2 and idx[1] == 0:
                    return [p[0] for p in self[idx[0]]]
                if isinstance(idx, tuple) and len(idx) == 2 and idx[1] == 1:
                    return [p[1] for p in self[idx[0]]]
                return super().__getitem__(idx)
        return NumpyArrayMock(lst)

    @staticmethod
    def linspace(start, stop, num=50, endpoint=True, retstep=False, dtype=None, axis=0):
        if num == 1:
            return [start]
        step = (stop - start) / (num - 1)
        return [start + i * step for i in range(num)]

    @staticmethod
    def vstack(tup):
        return list(zip(*tup))

    @staticmethod
    def round(a):
        class Wrapper:
            def astype(self, t):
                return [[int(round(x)), int(round(y))] for x, y in a]
        return Wrapper()

    class ndarray:
        pass

    float32 = "float32"

    @staticmethod
    def zeros(shape, dtype=None):
        return []

    @staticmethod
    def empty(shape):
        return []

sys.modules['numpy'] = MockNP()
sys.modules['scipy'] = MagicMock()
sys.modules['scipy.spatial'] = MagicMock()
sys.modules['data_models'] = MagicMock()
sys.modules['pydantic'] = MagicMock()

def test_bresenham():
    from slam_engine import SLAMEngine
    engine = SLAMEngine()

    start = [200, 200]

    def baseline(start, end):
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        steps = int(max(dx, dy))
        if steps == 0:
            return [[x0, y0]]
        xs = MockNP.linspace(x0, x1, steps + 1)
        ys = MockNP.linspace(y0, y1, steps + 1)
        points = MockNP.vstack((xs, ys))
        return MockNP.round(points).astype(int)

    test_cases = [
        ([200, 200], [200, 200]),
        ([200, 200], [210, 200]),
        ([200, 200], [200, 210]),
        ([200, 200], [210, 210]),
        ([200, 200], [190, 200]),
        ([200, 200], [200, 190]),
        ([200, 200], [190, 190]),
        ([200, 200], [210, 190]),
        ([200, 200], [190, 210]),
        ([200, 200], [215, 205]),
        ([200, 200], [205, 215])
    ]

    random.seed(42)
    for _ in range(100):
        angle = random.uniform(0, 2 * math.pi)
        dist = random.uniform(10, 150)
        end = [
            int(start[0] + math.cos(angle) * dist),
            int(start[1] + math.sin(angle) * dist)
        ]
        test_cases.append((start, end))

    for start, end in test_cases:
        base_res = baseline(start, end)
        opt_res = engine._bresenham_line(start, end)

        assert len(base_res) > 0
        assert len(opt_res) > 0

        assert list(opt_res[0]) == list(start), f"Start mismatch: {list(opt_res[0])} != {start}"
        assert list(opt_res[-1]) == list(end), f"End mismatch: {list(opt_res[-1])} != {end}"

        for i in range(1, len(opt_res)):
            p1 = opt_res[i-1]
            p2 = opt_res[i]
            dx = abs(p2[0] - p1[0])
            dy = abs(p2[1] - p1[1])
            assert dx <= 1 and dy <= 1, f"Gap in line: {p1} to {p2}"

    print("All raycasting tests passed.")

if __name__ == "__main__":
    test_bresenham()
