"""Open an archived shm_trace snapshot whether it is still `X.json` or has been
compacted to `X.json.zst` (archive_compact.sh). Campaign CSVs keep the `.json`
path; readers that go through here keep working after compaction.

    from snapio import load_json
    d = load_json(row["snapshot"])
"""
import gzip, json, os, subprocess


def resolve(path):
    for cand in (path, path + ".zst", path + ".gz"):
        if os.path.exists(cand):
            return cand
    base = path
    for ext in (".zst", ".gz"):
        if base.endswith(ext):
            base = base[: -len(ext)]
    if os.path.exists(base):
        return base
    raise FileNotFoundError(path)


def read_bytes(path):
    p = resolve(path)
    if p.endswith(".zst"):
        try:
            import zstandard
            with open(p, "rb") as fh:
                return zstandard.ZstdDecompressor().stream_reader(fh).read()
        except ImportError:
            return subprocess.run(["zstd", "-dc", "-q", p], capture_output=True, check=True).stdout
    if p.endswith(".gz"):
        with gzip.open(p, "rb") as fh:
            return fh.read()
    with open(p, "rb") as fh:
        return fh.read()


def load_json(path):
    return json.loads(read_bytes(path))


def open_text(path):
    """A text file object over the (possibly compacted) snapshot."""
    import io
    return io.StringIO(read_bytes(path).decode("utf-8", "replace"))
