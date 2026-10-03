#!/usr/bin/env python3
"""Save one four-pane LiDAR/OAK navigation preview from the running dashboard."""
import argparse
from pathlib import Path
from urllib.request import urlopen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8080/snapshot.png")
    parser.add_argument("--output", type=Path, default=Path("artifacts/navigation_snapshot.png"))
    args = parser.parse_args()
    with urlopen(args.url, timeout=10) as response:
        if response.headers.get_content_type() != "image/png":
            raise RuntimeError("Dashboard did not return a PNG")
        png = response.read()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(png)
    print(f"Saved {args.output} ({len(png)} bytes)")


if __name__ == "__main__":
    main()
