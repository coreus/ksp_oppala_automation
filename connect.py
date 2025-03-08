
import krpc
from krpc.client import Client
ADDRESS = '172.20.16.1'

class Connect:
    def start(name) -> Client:

        return krpc.connect(
            name=name,
            address=ADDRESS,
            rpc_port=50000, stream_port=50001)
