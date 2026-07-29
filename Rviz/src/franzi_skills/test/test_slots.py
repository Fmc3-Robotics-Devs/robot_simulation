"""The outfeed slot book."""

import pytest

from franzi_skills.slots import SlotBook, TrayFull


def test_claims_in_order_until_full():
    book = SlotBook(rows=1, columns=2, pitch=(0.1, 0.1))
    first, _ = book.claim()
    second, _ = book.claim()
    assert first != second
    with pytest.raises(TrayFull):
        book.claim()


def test_offsets_are_centred():
    book = SlotBook(rows=1, columns=2, pitch=(0.1, 0.12))
    _, (x0, y0) = book.claim()
    _, (x1, y1) = book.claim()
    assert x0 == x1 == 0.0
    assert y0 == pytest.approx(-0.06)
    assert y1 == pytest.approx(0.06)


def test_named_claim_and_release():
    book = SlotBook(rows=2, columns=2)
    name, _ = book.claim("1_1")
    assert name == "1_1"
    with pytest.raises(TrayFull):
        book.claim("1_1")
    book.release("1_1")
    assert "1_1" in book.free


def test_single_slot_tray():
    book = SlotBook(rows=1, columns=1)
    _, offset = book.claim()
    assert offset == (0.0, 0.0)


def test_unknown_slot_is_an_error():
    with pytest.raises(KeyError):
        SlotBook(rows=1, columns=1).offset("3_3")
