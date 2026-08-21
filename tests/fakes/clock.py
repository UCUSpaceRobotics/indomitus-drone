class FakeClock:
    def __init__(self, now: float = 0.0):
        self.value = now

    def now(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        if seconds < 0:
            raise ValueError("clock cannot move backward")
        self.value += seconds
