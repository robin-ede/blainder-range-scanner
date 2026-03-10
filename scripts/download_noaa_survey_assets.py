#!/usr/bin/env python3
"""Scrape and optionally download files linked from an NOAA survey page."""

from __future__ import annotations

import argparse
import json
import urllib.parse
import urllib.request
from pathlib import Path

from bs4 import BeautifulSoup


def fetch_text(url: str) -> str:
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req, timeout=60) as resp:
        return resp.read().decode("utf-8", errors="replace")


def scrape_assets(page_url: str):
    text = fetch_text(page_url)
    soup = BeautifulSoup(text, "html.parser")
    assets = []
    for link in soup.find_all("a", href=True):
        href = link.get("href")
        if not href:
            continue
        href = urllib.parse.urljoin(page_url, href)
        label = link.get_text(" ", strip=True)
        if any(
            ext in href
            for ext in [
                ".gz",
                ".xml",
                ".json",
                ".txt",
                ".all",
                ".mb201",
                ".bag",
                ".tar",
            ]
        ):
            assets.append({"label": label, "url": href})
    # de-dup preserve order
    seen = set()
    deduped = []
    for item in assets:
        if item["url"] in seen:
            continue
        seen.add(item["url"])
        deduped.append(item)
    return deduped


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--page", required=True, help="NOAA survey page URL")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--download-first", action="store_true")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    assets = scrape_assets(args.page)
    manifest_path = output_dir / "survey_assets.json"
    manifest_path.write_text(json.dumps(assets, indent=2), encoding="utf-8")
    print(f"Wrote {len(assets)} assets to {manifest_path}")

    if args.download_first and assets:
        first = assets[0]
        filename = Path(urllib.parse.urlparse(first["url"]).path).name
        dest = output_dir / filename
        urllib.request.urlretrieve(first["url"], dest)
        print(f"Downloaded {first['url']} -> {dest}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
