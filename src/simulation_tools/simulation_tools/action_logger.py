import sqlite3
import time
import json
from threading import Lock

class ActionLogger:
    """Simple SQLite-based logger for web UI actions."""

    def __init__(self, db_path: str):
        self.db_path = db_path
        self._lock = Lock()
        self._conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self._initialize()

    def _initialize(self):
        with self._conn:
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS actions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    action TEXT NOT NULL,
                    details TEXT
                )
                """
            )

    def log(self, action: str, details=None):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        detail_str = json.dumps(details) if details is not None else None
        with self._lock:
            self._conn.execute(
                "INSERT INTO actions (timestamp, action, details) VALUES (?, ?, ?)",
                (timestamp, action, detail_str)
            )
            self._conn.commit()

    def close(self):
        """Close the underlying SQLite connection."""
        if self._conn is not None:
            with self._lock:
                self._conn.close()
                self._conn = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False
