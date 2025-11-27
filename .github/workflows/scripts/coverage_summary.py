import json
import os
import xml.etree.ElementTree as ET


def rate_from_cobertura(path: str):
    try:
        root = ET.parse(path).getroot()
    except Exception:
        return None
    if "line-rate" in root.attrib:
        try:
            return float(root.attrib["line-rate"]) * 100.0
        except Exception:
            pass
    if "lines-valid" in root.attrib and "lines-covered" in root.attrib:
        try:
            lv = float(root.attrib["lines-valid"])
            lc = float(root.attrib["lines-covered"])
            return (lc / lv) * 100.0 if lv else None
        except Exception:
            return None
    return None


def rate_from_pycoverage(path: str):
    try:
        root = ET.parse(path).getroot()
    except Exception:
        return None
    try:
        return float(root.attrib.get("line-rate", 0)) * 100.0
    except Exception:
        return None


def rate_from_lcov(path: str):
    """
    Compute line-rate from an lcov tracefile (coverage.info).
    Falls back silently if the file cannot be parsed.
    """
    try:
        total_lines = 0
        hit_lines = 0
        with open(path) as f:
            for line in f:
                if line.startswith("LF:"):
                    total_lines += int(line.split(":", 1)[1])
                elif line.startswith("LH:"):
                    hit_lines += int(line.split(":", 1)[1])
        if total_lines:
            return (hit_lines / total_lines) * 100.0
    except Exception:
        return None
    return None


def rate_from_gcovr_html(path: str):
    """
    Try to extract line coverage from a gcovr HTML report.
    Looks for patterns like 'Lines: XX%' in the summary table.
    """
    import re

    try:
        with open(path, encoding="utf-8", errors="ignore") as f:
            content = f.read()
        # common patterns in gcovr HTML summaries
        patterns = [
            r"Lines:\s*([0-9]+(?:\.[0-9]+)?)%",
            r"Line Coverage:\s*([0-9]+(?:\.[0-9]+)?)%",
        ]
        for pat in patterns:
            match = re.search(pat, content)
            if match:
                return float(match.group(1))
    except Exception:
        return None
    return None


def rate_from_js(path: str):
    try:
        with open(path) as f:
            data = json.load(f)
        return float(data["total"]["lines"]["pct"])
    except Exception:
        return None


def main():
    summary = []
    cpp = None
    # Prefer lcov, then Cobertura XML, finally try to scrape the HTML if that's all we have.
    if os.path.exists("coverage.info"):
        cpp = rate_from_lcov("coverage.info")
    if cpp is None and os.path.exists("coverage-cpp.xml"):
        cpp = rate_from_cobertura("coverage-cpp.xml")
    if cpp is None and os.path.exists("coverage-cpp.html"):
        cpp = rate_from_gcovr_html("coverage-cpp.html")
    py = rate_from_pycoverage("coverage-python.xml") if os.path.exists("coverage-python.xml") else None
    js_path = "www/webroot/js/coverage/coverage-summary.json"
    js = rate_from_js(js_path) if os.path.exists(js_path) else None

    if cpp is not None:
        summary.append(f"- C++ coverage: {cpp:.1f}%")
    if py is not None:
        summary.append(f"- Python coverage: {py:.1f}%")
    if js is not None:
        summary.append(f"- JS coverage: {js:.1f}%")

    if summary:
        out_path = os.environ.get("GITHUB_STEP_SUMMARY", "coverage-summary.md")
        with open(out_path, "a") as f:
            f.write("## Coverage summary\n")
            for line in summary:
                f.write(line + "\n")


if __name__ == "__main__":
    main()
