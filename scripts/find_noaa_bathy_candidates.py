#!/usr/bin/env python3
"""Find shallow NOAA multibeam surveys suitable for validation.

Queries NOAA's ArcGIS multibeam footprint service, then optionally enriches
results by scraping each survey page for project name, depth range, and bounds.

The ranking favors:
- NOAA/NCBO sources
- shallow depth ranges close to the current synthetic regime
- compact extents that are easier to crop into a Blender mesh
- modern surveys with multibeam instruments

Examples:
  python scripts/find_noaa_bathy_candidates.py
  python scripts/find_noaa_bathy_candidates.py --source ncbo --limit 15
  python scripts/find_noaa_bathy_candidates.py --survey-id LD-11-04-CB-HACR
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional

from bs4 import BeautifulSoup


FEATURE_QUERY_URL = (
    "https://services2.arcgis.com/C8EMgrsFcRFL6LrL/arcgis/rest/services/"
    "multibeam_footprints/FeatureServer/0/query"
)


DEFAULT_OUT_FIELDS = [
    "SURVEY_ID",
    "SURVEY_YEAR",
    "PLATFORM",
    "SOURCE",
    "INSTRUMENT",
    "DOWNLOAD_URL",
]


@dataclass
class SurveyCandidate:
    survey_id: str
    survey_year: Optional[int]
    platform: str
    source: str
    instrument: str
    download_url: str
    project: Optional[str] = None
    min_depth_m: Optional[float] = None
    max_depth_m: Optional[float] = None
    min_sonar_depth_m: Optional[float] = None
    max_sonar_depth_m: Optional[float] = None
    north: Optional[float] = None
    south: Optional[float] = None
    west: Optional[float] = None
    east: Optional[float] = None
    score: float = 0.0
    reasons: List[str] = field(default_factory=list)

    @property
    def depth_span_m(self) -> Optional[float]:
        if self.min_depth_m is None or self.max_depth_m is None:
            return None
        return self.max_depth_m - self.min_depth_m

    @property
    def bbox_area_deg2(self) -> Optional[float]:
        if None in (self.north, self.south, self.west, self.east):
            return None
        assert self.east is not None
        assert self.west is not None
        assert self.north is not None
        assert self.south is not None
        east = float(self.east)
        west = float(self.west)
        north = float(self.north)
        south = float(self.south)
        return max(0.0, east - west) * max(0.0, north - south)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "survey_id": self.survey_id,
            "survey_year": self.survey_year,
            "platform": self.platform,
            "source": self.source,
            "instrument": self.instrument,
            "project": self.project,
            "download_url": self.download_url,
            "min_depth_m": self.min_depth_m,
            "max_depth_m": self.max_depth_m,
            "depth_span_m": self.depth_span_m,
            "min_sonar_depth_m": self.min_sonar_depth_m,
            "max_sonar_depth_m": self.max_sonar_depth_m,
            "bounds": {
                "north": self.north,
                "south": self.south,
                "west": self.west,
                "east": self.east,
            },
            "bbox_area_deg2": self.bbox_area_deg2,
            "score": self.score,
            "reasons": self.reasons,
        }


def fetch_json(url: str, params: Dict[str, Any]) -> Dict[str, Any]:
    query = urllib.parse.urlencode(params)
    req = urllib.request.Request(
        f"{url}?{query}", headers={"User-Agent": "Mozilla/5.0"}
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.load(resp)


def fetch_text(url: str) -> str:
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req, timeout=30) as resp:
        return resp.read().decode("utf-8", errors="replace")


def arcgis_query(where: str, limit: int = 100) -> List[SurveyCandidate]:
    params = {
        "where": where,
        "outFields": ",".join(DEFAULT_OUT_FIELDS),
        "returnGeometry": "false",
        "resultRecordCount": str(limit),
        "orderByFields": "SURVEY_YEAR DESC",
        "f": "pjson",
    }
    data = fetch_json(FEATURE_QUERY_URL, params)
    out: List[SurveyCandidate] = []
    for feature in data.get("features", []):
        attrs = feature.get("attributes", {})
        out.append(
            SurveyCandidate(
                survey_id=attrs.get("SURVEY_ID") or "",
                survey_year=attrs.get("SURVEY_YEAR"),
                platform=attrs.get("PLATFORM") or "",
                source=attrs.get("SOURCE") or "",
                instrument=attrs.get("INSTRUMENT") or "",
                download_url=attrs.get("DOWNLOAD_URL") or "",
            )
        )
    return out


def soup_table_map(text: str) -> Dict[str, str]:
    soup: Any = BeautifulSoup(text, "html.parser")
    mapping: Dict[str, str] = {}
    for row in soup.find_all("tr"):
        cells = row.find_all("td")
        cleaned = [cell.get_text(" ", strip=True) for cell in cells]
        if len(cleaned) < 2:
            continue
        # Pair adjacent cells: label/value, label/value
        for i in range(0, len(cleaned) - 1, 2):
            key = cleaned[i].strip()
            value = cleaned[i + 1].strip()
            if key and value and key not in mapping:
                mapping[key] = value
    return mapping


def extract_float(field_map: Dict[str, str], label: str) -> Optional[float]:
    value = field_map.get(label)
    if not value:
        return None
    match = re.search(r"[-+]?[0-9]*\.?[0-9]+", value)
    if not match:
        return None
    try:
        return float(match.group(0))
    except ValueError:
        return None


def enrich_candidate(candidate: SurveyCandidate) -> SurveyCandidate:
    if not candidate.download_url:
        return candidate
    try:
        text = fetch_text(candidate.download_url)
    except Exception as exc:  # pragma: no cover - network failure path
        candidate.reasons.append(f"page fetch failed: {exc}")
        return candidate

    field_map = soup_table_map(text)

    candidate.project = field_map.get("Project:") or field_map.get("Project")
    candidate.min_depth_m = extract_float(field_map, "Minimum Depth")
    candidate.max_depth_m = extract_float(field_map, "Maximum Depth")
    candidate.min_sonar_depth_m = extract_float(field_map, "Minimum Sonar Depth")
    candidate.max_sonar_depth_m = extract_float(field_map, "Maximum Sonar Depth")
    candidate.north = extract_float(field_map, "Northern Extent:") or extract_float(
        field_map, "Northern Extent"
    )
    candidate.south = extract_float(field_map, "Southern Extent:") or extract_float(
        field_map, "Southern Extent"
    )
    candidate.west = extract_float(field_map, "Western Extent:") or extract_float(
        field_map, "Western Extent"
    )
    candidate.east = extract_float(field_map, "Eastern Extent:") or extract_float(
        field_map, "Eastern Extent"
    )
    return candidate


def rank_candidate(candidate: SurveyCandidate) -> SurveyCandidate:
    score = 0.0
    reasons: List[str] = []

    source_l = candidate.source.lower()
    project_l = (candidate.project or "").lower()
    platform_l = candidate.platform.lower()
    instrument_l = candidate.instrument.lower()

    if "chesapeake bay office" in source_l:
        score += 35
        reasons.append("NCBO shallow-water source")
    elif "noaa" in source_l:
        score += 20
        reasons.append("NOAA source")

    if any(
        word in project_l
        for word in ["reef", "oyster", "harbor", "bay", "shoal", "coastal"]
    ):
        score += 20
        reasons.append("project suggests shallow coastal morphology")

    if (
        "em2040" in instrument_l
        or "8125" in instrument_l
        or "r2sonic 2024" in instrument_l
    ):
        score += 12
        reasons.append("good shallow-water multibeam instrument")
    elif "7125" in instrument_l or "em710" in instrument_l:
        score += 8
        reasons.append("usable multibeam instrument")

    if candidate.survey_year is not None:
        if candidate.survey_year >= 2020:
            score += 8
            reasons.append("very recent")
        elif candidate.survey_year >= 2010:
            score += 5
            reasons.append("modern survey")

    if candidate.min_depth_m is not None and candidate.max_depth_m is not None:
        min_d = candidate.min_depth_m
        max_d = candidate.max_depth_m
        span = max_d - min_d

        if max_d <= 20.0:
            score += 25
            reasons.append("entire depth range is shallow")
        elif max_d <= 30.0:
            score += 15
            reasons.append("mostly shallow depth range")
        elif max_d <= 60.0:
            score += 5
            reasons.append("workable depth range after cropping")
        else:
            score -= 10
            reasons.append("deeper survey; weaker match")

        if min_d <= 2.5:
            score += 10
            reasons.append("includes very shallow seabed")
        elif min_d <= 5.0:
            score += 5
            reasons.append("starts in shallow water")

        if span <= 20.0:
            score += 10
            reasons.append("compact depth span")
        elif span <= 50.0:
            score += 4
            reasons.append("moderate depth span")

        target_mid = 8.0
        mid = (min_d + max_d) / 2.0
        score -= min(abs(mid - target_mid), 25.0) * 0.5

    if candidate.bbox_area_deg2 is not None:
        area = candidate.bbox_area_deg2
        if area <= 0.001:
            score += 10
            reasons.append("compact footprint; easy to crop")
        elif area <= 0.01:
            score += 6
            reasons.append("moderate footprint")
        else:
            score -= 2
            reasons.append("larger footprint")

    if "look down" in platform_l or "potawaugh" in platform_l:
        score += 8
        reasons.append("small-platform nearshore survey")

    candidate.score = round(score, 2)
    candidate.reasons = reasons
    return candidate


def dedupe(candidates: Iterable[SurveyCandidate]) -> List[SurveyCandidate]:
    seen = {}
    for candidate in candidates:
        seen[candidate.survey_id] = candidate
    return list(seen.values())


def build_where_clause(source_mode: str, survey_id: Optional[str]) -> str:
    if survey_id:
        safe = survey_id.replace("'", "''")
        return f"SURVEY_ID = '{safe}'"
    if source_mode == "ncbo":
        return "SOURCE like '%NOAA Chesapeake Bay Office%' OR SURVEY_ID like '%CB%'"
    if source_mode == "nearshore":
        return (
            "(SOURCE like '%NOAA Chesapeake Bay Office%' OR SOURCE like '%NOAA%') "
            "AND SURVEY_YEAR >= 2010 "
            "AND (INSTRUMENT like '%EM2040%' OR INSTRUMENT like '%8125%' "
            "OR INSTRUMENT like '%7125%' OR INSTRUMENT like '%R2Sonic%')"
        )
    return "SOURCE like '%NOAA%' AND SURVEY_YEAR >= 2010"


def print_table(candidates: List[SurveyCandidate]) -> None:
    for idx, candidate in enumerate(candidates, start=1):
        depth = "unknown"
        if candidate.min_depth_m is not None and candidate.max_depth_m is not None:
            depth = f"{candidate.min_depth_m:.2f}-{candidate.max_depth_m:.2f} m"
        print(f"[{idx}] {candidate.survey_id}  score={candidate.score}")
        print(f"    year/platform : {candidate.survey_year} / {candidate.platform}")
        print(f"    source        : {candidate.source}")
        print(f"    instrument    : {candidate.instrument}")
        print(f"    project       : {candidate.project or 'unknown'}")
        print(f"    depth range   : {depth}")
        print(f"    url           : {candidate.download_url}")
        print(f"    why           : {', '.join(candidate.reasons[:5])}")
        print()


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        choices=["ncbo", "nearshore", "noaa"],
        default="nearshore",
        help="Candidate pool to query (default: nearshore).",
    )
    parser.add_argument("--survey-id", help="Inspect a single survey id exactly.")
    parser.add_argument(
        "--limit", type=int, default=20, help="Number of candidate footprints to fetch."
    )
    parser.add_argument(
        "--top", type=int, default=10, help="Number of ranked results to print."
    )
    parser.add_argument(
        "--json",
        dest="json_path",
        help="Optional path to write full ranked JSON output.",
    )
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    where = build_where_clause(args.source, args.survey_id)
    candidates = arcgis_query(where=where, limit=args.limit)
    if not candidates:
        print("No candidates found.", file=sys.stderr)
        return 1

    enriched = [rank_candidate(enrich_candidate(c)) for c in dedupe(candidates)]
    enriched.sort(key=lambda c: c.score, reverse=True)

    print(f"Query: {where}")
    print(f"Fetched {len(enriched)} unique candidates.\n")
    print_table(enriched[: args.top])

    if args.json_path:
        with open(args.json_path, "w", encoding="utf-8") as fh:
            json.dump([c.to_dict() for c in enriched], fh, indent=2)
        print(f"Wrote JSON to {args.json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
