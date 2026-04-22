
import configparser
import os
import krpc
from krpc.client import Client

_config = configparser.ConfigParser()
_config.read(os.path.join(os.path.dirname(__file__), '..', 'config.ini'))

class Connect:
    def start(name) -> Client:
        address = _config.get('krpc', 'address')
        rpc_port = _config.getint('krpc', 'rpc_port')
        stream_port = _config.getint('krpc', 'stream_port')

        return krpc.connect(
            name=name,
            address=address,
            rpc_port=rpc_port, stream_port=stream_port)
