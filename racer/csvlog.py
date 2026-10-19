import csv
from pathlib import Path


def open_csv_log(path, fields):
    """Open a CSV for appending. Writes the header only when the file is new."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size > 0:
        with open(path, newline="") as f:
            header = next(csv.reader(f), [])
        if header != fields:  # columns changed since the file was written: keep the old one aside
            path.replace(path.with_name(path.stem + "_old" + path.suffix))
    f = open(path, "a", newline="")
    writer = csv.DictWriter(f, fieldnames=fields)
    if f.tell() == 0:
        writer.writeheader()
        f.flush()
    return f, writer
