import glob
import json
import os
from typing import List, Optional, Tuple
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


def rate_from_js_lcov(path: str):
    # Reuse the lcov parser to compute JS coverage if coverage-summary.json is missing.
    return rate_from_lcov(path)


def parse_junit_summary(path: str) -> Optional[Tuple[int, int, int, int]]:
    """
    Return (tests, errors, failures, skipped) for a JUnit XML file.
    Handles both <testsuite> and <testsuites>.
    """
    try:
        root = ET.parse(path).getroot()
    except Exception:
        return None

    def accum(node) -> Tuple[int, int, int, int]:
        t = int(node.attrib.get("tests", 0))
        e = int(node.attrib.get("errors", 0))
        f = int(node.attrib.get("failures", 0))
        s = int(node.attrib.get("skipped", 0))
        return t, e, f, s

    if root.tag == "testsuite":
        return accum(root)
    if root.tag == "testsuites":
        total_t = total_e = total_f = total_s = 0
        for ts in root.findall("testsuite"):
            t, e, f, s = accum(ts)
            total_t += t
            total_e += e
            total_f += f
            total_s += s
        return total_t, total_e, total_f, total_s
    return None


def gather_junit_results(
    base_dir: str,
) -> Tuple[int, int, int, int, List[Tuple[str, int, int, int, int]]]:
    """
    Aggregate JUnit results under base_dir.
    Returns total tests/errors/failures/skipped and per-suite details.
    """
    total_tests = total_errors = total_failures = total_skipped = 0
    details: List[Tuple[str, int, int, int, int]] = []
    for path in glob.glob(os.path.join(base_dir, "**", "*.xml"), recursive=True):
        summary = parse_junit_summary(path)
        if summary is None:
            continue
        t, e, f, s = summary
        total_tests += t
        total_errors += e
        total_failures += f
        total_skipped += s
        details.append((os.path.relpath(path, base_dir), t, e, f, s))
    return total_tests, total_errors, total_failures, total_skipped, details


def highlight_if_issue(line: str, errors: int, failures: int, skipped: int) -> str:
    """
    Wrap the line in a red span if any non-zero errors/failures/skipped.
    """
    if errors > 0 or failures > 0 or skipped > 0:
        return f"<span style='color:red'>{line}</span>"
    return line


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
    if js is None:
        js_lcov = "www/webroot/js/coverage/lcov.info"
        if os.path.exists(js_lcov):
            js = rate_from_js_lcov(js_lcov)

    junit_base = "src/workspace/build/test_results"
    junit_total_line: Optional[str] = None
    junit_detail_lines: List[str] = []
    if os.path.isdir(junit_base):
        t, e, f, s, details = gather_junit_results(junit_base)
        if t > 0:
            total_line_raw = (
                f"- Tests: {t} run, {e} errors, {f} failures, {s} skipped "
                "(artifact: junit-test-results)"
            )
            junit_total_line = highlight_if_issue(total_line_raw, e, f, s)
            # List all suites (sorted) for clarity.
            for relpath, dt, de, df, ds in sorted(details):
                detail_raw = (
                    f"- {relpath}: {dt} tests, {de} errors, {df} failures, {ds} skipped"
                )
                junit_detail_lines.append(highlight_if_issue(detail_raw, de, df, ds))

    if cpp is not None:
        summary.append(f"- C++ coverage: {cpp:.1f}%")
    if py is not None:
        summary.append(f"- Python coverage: {py:.1f}%")
    if js is not None:
        summary.append(f"- JS coverage: {js:.1f}%")
    out_path = os.environ.get("GITHUB_STEP_SUMMARY", "coverage-summary.md")

    # Avoid duplicate coverage sections if the script is invoked multiple times in the same step.
    existing = ""
    if os.path.exists(out_path):
        try:
            with open(out_path, "r") as f:
                existing = f.read()
        except Exception:
            existing = ""

    if summary and "## Coverage summary" not in existing:
        with open(out_path, "a") as f:
            f.write("## Coverage summary\n")
            for line in summary:
                f.write(line + "\n")
            f.write("\n")

    if junit_total_line and "## Test results (JUnit)" not in existing:
        with open(out_path, "a") as f:
            f.write("## Test results (JUnit)\n")
            f.write(junit_total_line + "\n")
            for line in junit_detail_lines:
                f.write(line + "\n")

    if summary:
        out_path = os.environ.get("GITHUB_STEP_SUMMARY", "coverage-summary.md")
        with open(out_path, "a") as f:
            f.write("## Coverage summary\n")
            for line in summary:
                f.write(line + "\n")


if __name__ == "__main__":
    main()
