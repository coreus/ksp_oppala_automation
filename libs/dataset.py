class Dataset:
    def __init__(self,filename) -> None:
        self.filename = filename
        self.f = open(filename, "a")

    def log(self,data: str) -> None:
        self.f.write(data + "\n")

    def close(self) -> None:
        self.f.close()