import csv
from pathlib import Path


def open_csv_log(path, fields):
    """Open a CSV for appending. Writes the header only when the file is new."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    f = open(path, "a", newline="")
    writer = csv.DictWriter(f, fieldnames=fields)
    if f.tell() == 0:
        writer.writeheader()
        f.flush()
    return f, writer
