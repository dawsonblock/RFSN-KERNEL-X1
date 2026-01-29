from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from typing import Any, Dict, Iterable, List, Optional


@dataclass(frozen=True)
class TraceRecord:
    t: float
    tag: str
    payload: Dict[str, Any]


def dumps_jsonl(records: Iterable[TraceRecord]) -> str:
    lines = []
    for r in records:
        lines.append(json.dumps({"t": r.t, "tag": r.tag, "payload": r.payload}, sort_keys=True))
    return "\n".join(lines) + ("\n" if lines else "")


def loads_jsonl(text: str) -> List[TraceRecord]:
    out: List[TraceRecord] = []
    for line in text.splitlines():
        if not line.strip():
            continue
        obj = json.loads(line)
        out.append(TraceRecord(t=float(obj["t"]), tag=str(obj["tag"]), payload=dict(obj["payload"])))
    return out
