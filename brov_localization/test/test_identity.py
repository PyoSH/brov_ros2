import uuid

from brov_localization.identity import AlignmentIdGenerator


def test_alignment_ids_are_unique_across_generations_and_node_restarts() -> None:
    first_boot = AlignmentIdGenerator()
    second_boot = AlignmentIdGenerator()
    identifiers = {
        first_boot.new(),
        first_boot.new(),
        second_boot.new(),
        second_boot.new(),
    }
    assert len(identifiers) == 4
    for identifier in identifiers:
        assert str(uuid.UUID(identifier)) == identifier
