class Logger:
    def __init__(self):
        self.log_count = 0
    
    def log(self, msg):
        if self.log_count % 10 == 0:
            print(msg)
        self.log_count += 1