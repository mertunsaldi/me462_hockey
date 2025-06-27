import os, sys, types, pytest
from collections import deque

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

serial_stub = types.SimpleNamespace(
    SerialException=Exception,
    tools=types.SimpleNamespace(list_ports=types.SimpleNamespace(comports=lambda: [])),
)
sys.modules.setdefault("serial", serial_stub)
sys.modules.setdefault("serial.tools", serial_stub.tools)
sys.modules.setdefault("serial.tools.list_ports", serial_stub.tools.list_ports)

if "numpy" not in sys.modules:
    np_stub = types.SimpleNamespace(
        array=lambda *a, **k: a[0],
        stack=lambda *a, **k: a[0],
        linalg=types.SimpleNamespace(inv=lambda x: x),
        linspace=lambda *a, **k: [],
        clip=lambda *a, **k: [],
        sqrt=lambda x: 0.0,
        int32=int,
        ndarray=object,
    )
    sys.modules["numpy"] = np_stub

from ball_example.gadgets import PlotClock

class StubMaster:
    def __init__(self):
        self.sent = []
        self.lines = deque()

    def send_command(self, cmd: str) -> None:
        self.sent.append(cmd)

    def get_lines(self):
        lines = list(self.lines)
        self.lines.clear()
        return lines

    def unget_lines(self, lines):
        for line in reversed(lines):
            self.lines.appendleft(line)


def test_query_value_preserves_other_lines():
    master = StubMaster()
    clock = PlotClock()
    clock.device_id = 0
    clock.master = master

    master.lines.append("P1:foo:bar")
    master.lines.append("P0:resp:42")
    master.lines.append("P1:baz:99")

    value = clock._query_value("p.getFoo()", timeout=0.1)
    assert value == "42"
    assert list(master.lines) == ["P1:foo:bar", "P1:baz:99"]

