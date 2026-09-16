# Modified by the Agent-as-Policy (AgP) authors, 2026.
# Original: graph-as-policy @ f09e37e7755d0a908c1b6cf136edf13292b730c0
#   https://github.com/graph-robots/graph-as-policy  (Apache-2.0, see
#   third_party/graph-as-policy/LICENSE)
# Changes: vendoring trim only.  The eager ``gap.connector.collector`` and
#   ``gap.connector.sim`` imports and their three ``__all__`` entries
#   (``DataCollector``, ``SimConnector``, ``sim``) were removed so the
#   real-YAM subset vendored here imports without h5py or the sim stack.
#   The module docstring is upstream's and still shows a sim example.
#   Nothing else changed; this file is otherwise byte-identical to
#   upstream.  (+0/-5, introduced by the vendoring itself)
#   Per-file diffstat and the full A/B/C classification in
#   third_party/graph-as-policy/UPSTREAM.md.
"""gap.connector — backends that own an environment and its tools.

A connector binds an environment instance (sim or real) to the runtime:
it registers the ``robot.*`` / ``sim.*`` tools on a fresh ToolRegistry,
assembles :class:`gap.types.Observation` snapshots, and exposes
capabilities + ground-truth world snapshots where the backend supports
them.

    import gap
    conn = gap.connector.sim("libero", task="libero_object/0")
    result = gap.execute(graph, conn)   # open-robot-skills auto-discovered
"""

from __future__ import annotations

from gap.connector.core import Capabilities, Connector
from gap.connector.real import RealConnector, real

__all__ = [
    "Capabilities",
    "Connector",
    "RealConnector",
    "real",
]
