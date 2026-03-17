import sys
import time
import math
import random

# We don't need SLAMEngine, we can just isolate the algorithm.

class MockNP:
    @staticmethod
    def array(lst, dtype=None):
        return lst
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

mock_np = MockNP()

def baseline(start, end):
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    steps = int(max(dx, dy))
    if steps == 0:
        return [[x0, y0]]
    xs = mock_np.linspace(x0, x1, steps + 1)
    ys = mock_np.linspace(y0, y1, steps + 1)
    points = mock_np.vstack((xs, ys))
    return mock_np.round(points).astype(int)

def optimized(start, end):
    x0, y0 = start
    x1, y1 = end

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    x, y = x0, y0

    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1

    points = []

    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append([x, y])
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
        points.append([x, y])
    else:
        err = dy / 2.0
        while y != y1:
            points.append([x, y])
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
        points.append([x, y])

    # Simulate return as numpy array, using a list with shape attribute
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

    return NumpyArrayMock(points)


def run_benchmark(func, iterations=20000):
    start_pixel = [200, 200]

    random.seed(42)
    end_pixels = []
    for _ in range(iterations):
        angle = random.uniform(0, 2 * math.pi)
        dist = random.uniform(10, 150)
        end_pixels.append([
            int(start_pixel[0] + math.cos(angle) * dist),
            int(start_pixel[1] + math.sin(angle) * dist)
        ])

    start_time = time.time()
    for end_pixel in end_pixels:
        func(start_pixel, end_pixel)
    end_time = time.time()

    return end_time - start_time

if __name__ == "__main__":
    # Warmup
    run_benchmark(baseline, 100)
    run_benchmark(optimized, 100)

    # Benchmark
    time_base = run_benchmark(baseline)
    time_opt = run_benchmark(optimized)

    print(f"Baseline time: {time_base:.4f} s")
    print(f"Optimized time: {time_opt:.4f} s")
    if time_base > 0:
        print(f"Improvement: {(time_base - time_opt) / time_base * 100:.2f}%")
