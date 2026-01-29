from rfsn_kernel.sim_harness import run_sim
from rfsn_kernel.trace import loads_jsonl, dumps_jsonl

def test_trace_is_deterministic_on_replay():
    text1, records1 = run_sim()

    # Parse and re-dump => should match exactly because we used sort_keys=True and stable structures
    parsed = loads_jsonl(text1)
    text2 = dumps_jsonl(parsed)

    assert text1 == text2

def test_trace_contains_stop_and_estop():
    text, records = run_sim()
    tags = [r.tag for r in records]
    assert "monitor" in tags
    assert "controller" in tags

    # Ensure we saw STOP window and E_STOP window
    levels = [r.payload.get("level") for r in records if r.tag == "monitor"]
    assert "STOP" in levels
    assert "E_STOP" in levels
