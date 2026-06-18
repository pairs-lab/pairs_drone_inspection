"""Tests for perception.ocr.parse_label_text — pure regex, no model needed."""
from perception.ocr import parse_label_text


def test_parse_partno_qty():
    lines = ["GR LABEL", "Part No: PN-A01", "Qty: 11"]
    pn, qty = parse_label_text(lines)
    assert pn == "PN-A01" and qty == 11


def test_parse_tolerates_noise_and_case():
    lines = ["part no  pn-b03", "QTY :  19", "garbage"]
    pn, qty = parse_label_text(lines)
    assert pn == "PN-B03" and qty == 19


def test_parse_missing_returns_none():
    pn, qty = parse_label_text(["nothing here"])
    assert pn is None and qty is None


def test_parse_with_period_colon():
    lines = ["Part No.: PN-C06", "Qty: 5"]
    pn, qty = parse_label_text(lines)
    assert pn == "PN-C06" and qty == 5
