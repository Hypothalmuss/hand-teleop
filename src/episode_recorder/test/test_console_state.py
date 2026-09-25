from episode_recorder.console_state import ESC, ConsoleState


def test_key_actions():
    st = ConsoleState(seeds=[11, 22])
    assert st.handle(ESC) == ("estop", {})
    assert st.handle("c") == ("clear", {})
    assert st.handle("r") == ("reset", {"seed": 11})
    assert st.handle("r") == ("reset", {"seed": 22})
    assert st.handle("r") == ("reset", {"seed": 11})  # wraps around
    assert st.handle(" ") == ("start", {})
    st.recording = True
    assert st.handle("r") is None  # no reset while recording
    assert st.handle("s") is None and st.success_marked
    assert st.handle(" ") == ("stop", {"success": True, "discard": False})
    assert st.handle("d") == ("stop", {"success": False, "discard": True})
    st.recording = False
    assert st.handle("d") is None
    assert st.handle("x") is None


def test_status_line():
    st = ConsoleState(seeds=[5])
    st.handle("r")
    st.estop = True
    line = st.status_line()
    assert "E-STOP" in line and "seed 5" in line
