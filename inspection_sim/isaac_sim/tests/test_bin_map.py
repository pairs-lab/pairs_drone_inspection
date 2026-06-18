from sim.bin_map import generate_bin_map, validate_bin_map, BIN_IDS

def test_has_18_bins():
    m = generate_bin_map()
    assert len(m) == 18
    assert set(m.keys()) == set(BIN_IDS)

def test_bin_id_format():
    assert "A1" in BIN_IDS and "C6" in BIN_IDS

def test_each_bin_has_required_fields():
    m = generate_bin_map()
    for b in m.values():
        assert {"pallet_pose", "scan_pose", "part_no", "qty"} <= b.keys()
        assert len(b["scan_pose"]["position"]) == 3
        assert isinstance(b["qty"], int)

def test_validate_accepts_generated():
    assert validate_bin_map(generate_bin_map()) is True

def test_validate_rejects_wrong_count():
    bad = generate_bin_map(); bad.pop("A1")
    import pytest
    with pytest.raises(ValueError):
        validate_bin_map(bad)
