"""Make the package importable for plain `pytest` runs from any directory (no install)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
