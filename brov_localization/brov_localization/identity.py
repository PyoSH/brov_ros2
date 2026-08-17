"""Boot-scoped, standards-compliant localization alignment identities."""

from __future__ import annotations

from dataclasses import dataclass, field
import uuid


@dataclass
class AlignmentIdGenerator:
    """Create UUIDs that cannot repeat across node boots or generations."""

    boot_id: uuid.UUID = field(default_factory=uuid.uuid4)
    _generation: int = field(default=0, init=False)

    def new(self) -> str:
        """Return the next alignment UUID in this boot-unique namespace."""
        self._generation += 1
        return str(uuid.uuid5(self.boot_id, f"alignment:{self._generation}"))
