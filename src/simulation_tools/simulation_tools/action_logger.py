import sqlite3
import time
import json
import logging
from threading import Lock

class ActionLogger:
    """Simple SQLite-based logger for web UI actions."""

    def __init__(self, db_path: str, raise_errors: bool = False):
        self.db_path = db_path
        self.raise_errors = raise_errors
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
            try:
                if self._conn is None:
                    raise sqlite3.Error("Database connection is closed")
                self._conn.execute(
                    "INSERT INTO actions (timestamp, action, details) VALUES (?, ?, ?)",
                    (timestamp, action, detail_str)
                )
                self._conn.commit()
            except sqlite3.Error as e:
                logging.error(f"Failed to log action: {e}")
                if self.raise_errors:
                    raise

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

    def get_recent_actions(self, limit: int = 100):
        """Return the most recent actions as a list of dictionaries."""
        with self._lock:
            if self._conn is None:
                raise sqlite3.Error("Database connection is closed")
            cursor = self._conn.execute(
                "SELECT timestamp, action, details FROM actions ORDER BY id DESC LIMIT ?",
                (limit,),
            )
            rows = cursor.fetchall()

        actions = []
        for timestamp, action, details in rows:
            actions.append({
                "timestamp": timestamp,
                "action": action,
                "details": json.loads(details) if details else None,
            })
        return actions
