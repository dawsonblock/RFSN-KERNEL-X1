from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

from .controller_types import CapabilityLease, MaskedCommand, ControlSpace


@dataclass(frozen=True)
class ArbiterDecision:
    ok: bool
    reason: str
    selected_by_space: Dict[ControlSpace, MaskedCommand]


def arbitrate_multi(
    *,
    lease: CapabilityLease,
    proposals: List[MaskedCommand],
) -> ArbiterDecision:
    """
    Per-space deterministic arbitration:
      - safety overrides within that space
      - else only primary authority may command
      - strict: at most one eligible proposal per space
    """
    if lease.primary_authority is None:
        return ArbiterDecision(False, "No primary_authority map in lease", {})

    by_space: Dict[ControlSpace, List[MaskedCommand]] = {}
    for p in proposals:
        by_space.setdefault(p.space, []).append(p)

    out: Dict[ControlSpace, MaskedCommand] = {}

    for space, ps in sorted(by_space.items(), key=lambda kv: kv[0].value):
        safety = [p for p in ps if p.source == "safety"]
        if safety:
            if len(safety) != 1:
                return ArbiterDecision(False, f"Ambiguous safety proposals in {space.value}", {})
            out[space] = safety[0]
            continue

        primary = lease.primary_authority.get(space.value)
        if primary is None:
            return ArbiterDecision(False, f"No primary authority declared for {space.value}", {})

        eligible = [p for p in ps if p.source == primary]
        if len(eligible) == 0:
            continue  # hold that space
        if len(eligible) != 1:
            return ArbiterDecision(False, f"Ambiguous primary proposals in {space.value}", {})
        out[space] = eligible[0]

    if not out:
        return ArbiterDecision(False, "No proposals selected", {})

    return ArbiterDecision(True, "OK", out)
