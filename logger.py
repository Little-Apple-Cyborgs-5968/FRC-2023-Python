class Logger:
    def __init__(self, prefix: str = '', interval: float = 2) -> None:
        self.prefix = prefix
        self.interval = interval # log every this amount of time (seconds)
        self.last_logged = 0
    def stagger(self, now: float, message: str) -> None:
        if now - self.last_logged > self.interval:
            print(f'({now:.1f}) {self.prefix}: {message}')
            self.last_logged = now