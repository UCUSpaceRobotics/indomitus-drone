class SpyQueue:
    def __init__(self, error: Exception | None = None):
        self.values = []
        self.error = error
        self.calls = 0

    def put_nowait(self, value):
        self.calls += 1
        if self.error is not None:
            raise self.error
        self.values.append(value)
