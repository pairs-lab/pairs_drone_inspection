"""M5 — SAP Mock (SQLite-backed inventory store).

Provides a lightweight stand-in for SAP inventory data.
Seeds all 18 bins from bin_map ground truth but deliberately introduces
discrepancies on a few bins so the demo exercises both the "match" and
"discrepancy/alert" paths.

DISCREPANCY_BINS
----------------
The following bins have intentional mismatches between SAP (system record)
and bin_map ground truth (physical shelf):

  B2  — wrong quantity  : SAP qty = 99  (ground truth = 18)
  C4  — wrong part_no   : SAP part_no = "PN-WRONG" (ground truth = PN-C04)
  A5  — qty off by one  : SAP qty = 50  (ground truth = 15)

This gives three discrepancy scenarios:
  1. Qty mismatch (large delta)   → B2
  2. Part number mismatch         → C4
  3. Qty mismatch (small delta)   → A5

Usage
-----
    from backend.sap_mock import seed_from_bin_map, get_inventory, set_inventory

    seed_from_bin_map()                    # idempotent, skips if already seeded
    rec = get_inventory("A1")              # {"bin_id": "A1", "part_no": ..., "qty": ...}
    set_inventory("B2", "PN-B02", 18)      # re-verify / correct after physical check
"""

import sqlite3
import os
from typing import Optional, Dict, Any

# Path to the SQLite database (relative to repo root when running from there,
# or absolute when imported from elsewhere).
_DB_DEFAULT = os.path.join(os.path.dirname(__file__), "sap_mock.db")

# ---------------------------------------------------------------------------
# Intentional discrepancies for demo purposes (documented constant)
# ---------------------------------------------------------------------------
DISCREPANCY_BINS: Dict[str, Dict[str, Any]] = {
    "B2": {"part_no": "PN-B02", "qty": 99},       # wrong quantity
    "C4": {"part_no": "PN-WRONG", "qty": 26},     # wrong part_no
    "A5": {"part_no": "PN-A05", "qty": 50},       # qty off
}


def _connect(db_path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    return conn


def _ensure_table(conn: sqlite3.Connection) -> None:
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS inventory (
            bin_id   TEXT PRIMARY KEY,
            part_no  TEXT NOT NULL,
            qty      INTEGER NOT NULL
        )
        """
    )
    conn.commit()


def seed_from_bin_map(db_path: str = _DB_DEFAULT, force: bool = False) -> int:
    """Seed the inventory table from bin_map ground truth with intentional discrepancies.

    Args:
        db_path: Path to SQLite database file.
        force: If True, drop and re-seed even if data already exists.

    Returns:
        Number of rows inserted/replaced.
    """
    from sim.bin_map import load_bin_map

    bin_map = load_bin_map()
    conn = _connect(db_path)
    _ensure_table(conn)

    if not force:
        row = conn.execute("SELECT COUNT(*) FROM inventory").fetchone()
        if row[0] > 0:
            conn.close()
            return 0  # already seeded

    count = 0
    for bin_id, data in bin_map.items():
        if bin_id in DISCREPANCY_BINS:
            part_no = DISCREPANCY_BINS[bin_id]["part_no"]
            qty = DISCREPANCY_BINS[bin_id]["qty"]
        else:
            part_no = data["part_no"]
            qty = data["qty"]

        conn.execute(
            "INSERT OR REPLACE INTO inventory (bin_id, part_no, qty) VALUES (?, ?, ?)",
            (bin_id, part_no, qty),
        )
        count += 1

    conn.commit()
    conn.close()
    return count


def get_inventory(bin_id: str, db_path: str = _DB_DEFAULT) -> Optional[Dict[str, Any]]:
    """Fetch inventory record for a bin.

    Args:
        bin_id: Bin identifier, e.g. "A1".
        db_path: Path to SQLite database file.

    Returns:
        Dict {"bin_id": str, "part_no": str, "qty": int}, or None if not found.
    """
    conn = _connect(db_path)
    _ensure_table(conn)
    row = conn.execute(
        "SELECT bin_id, part_no, qty FROM inventory WHERE bin_id = ?", (bin_id,)
    ).fetchone()
    conn.close()
    if row is None:
        return None
    return {"bin_id": row["bin_id"], "part_no": row["part_no"], "qty": row["qty"]}


def set_inventory(bin_id: str, part_no: str, qty: int, db_path: str = _DB_DEFAULT) -> None:
    """Insert or update an inventory record (used for re-verification / SAP correction).

    Args:
        bin_id: Bin identifier.
        part_no: Correct part number after verification.
        qty: Correct quantity after verification.
        db_path: Path to SQLite database file.
    """
    conn = _connect(db_path)
    _ensure_table(conn)
    conn.execute(
        "INSERT OR REPLACE INTO inventory (bin_id, part_no, qty) VALUES (?, ?, ?)",
        (bin_id, part_no, qty),
    )
    conn.commit()
    conn.close()


if __name__ == "__main__":
    n = seed_from_bin_map(force=True)
    print(f"Seeded {n} rows. Discrepancy bins: {list(DISCREPANCY_BINS)}")
    for bid in ["A1", "B2", "C4", "A5"]:
        print(f"  {bid}: {get_inventory(bid)}")
