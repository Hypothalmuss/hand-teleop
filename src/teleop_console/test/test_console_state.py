from teleop_console.console_state import ESC, ConsoleState


def test_key_actions():
    st = ConsoleState(seeds=[11, 22])
    assert st.handle(ESC) == ("estop", {})
    assert st.handle("c") == ("clear", {})
    assert st.handle("r") == ("reset", {"seed": 11})
    assert st.handle("r") == ("reset", {"seed": 22})
    assert st.handle("r") == ("reset", {"seed": 11})  # wraps around
    assert st.handle("x") is None


def test_status_line():
    st = ConsoleState(seeds=[5])
    st.handle("r")
    st.estop = True
    line = st.status_line()
    assert "E-STOP" in line and "scene 5" in line
