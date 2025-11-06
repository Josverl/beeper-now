## reference 

class ESPNow(ESPNowBase, Iterator):
    def recv(self, timeout_ms: Optional[Any] = None) -> Union[Tuple[bytes, bytes], Tuple[None, None]]:
        ...
