#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Create a local ranking submission package from benchmark outputs.

The package layout is:

local_rank_ready/
  summary.csv
  README.txt
  <controller>/<task>_proof.zip
local_rank_ready.zip
"""

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path


def find_workspace_root():
    script_dir = Path(__file__).resolve().parent
    return script_dir.parents[2]


def find_proof_tool(workspace_root):
    candidates = [
        workspace_root / "tools" / "benchmark_proof_tool",
        workspace_root / "private" / "benchmark_proof_tool",
        workspace_root / "devel" / "lib" / "benchmark_utils" / "benchmark_proof_tool",
        workspace_root / "build" / "benchmark_utils" / "benchmark_proof_tool",
    ]
    for path in candidates:
        if path.is_file() and os.access(str(path), os.X_OK):
            return path
    found = shutil.which("benchmark_proof_tool")
    return Path(found) if found else None


def safe_name(text):
    out = []
    for ch in str(text):
        if ch.isalnum() or ch in "._-":
            out.append(ch)
        else:
            out.append("_")
    return "".join(out).strip("_") or "task"


def read_manifest_from_zip(proof_zip):
    with zipfile.ZipFile(proof_zip) as zf:
        return json.loads(zf.read("manifest.json").decode("utf-8"))


def verify_proof_zip(proof_tool, proof_zip):
    if proof_tool is None:
        return "unverified"
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        with zipfile.ZipFile(proof_zip) as zf:
            zf.extractall(tmp_path)
        cmd = [
            str(proof_tool),
            "--verify",
            "--csv", str(tmp_path / "result.csv"),
            "--manifest", str(tmp_path / "manifest.json"),
            "--signature", str(tmp_path / "signature.txt"),
        ]
        subprocess.run(cmd, check=True, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, text=True)
    return "ok"


def latest_csv_in_dir(path):
    csvs = [p for p in path.glob("*.csv") if p.name != "summary.csv"]
    if not csvs:
        return None
    return max(csvs, key=lambda p: p.stat().st_mtime)


def ensure_missing_proofs(results_dir, workspace_root, no_plot):
    analyzer = workspace_root / "src" / "benchmark_utils" / "scripts" / "benchmark_analyzer.py"
    if not analyzer.exists():
        raise RuntimeError("benchmark_analyzer.py not found")

    created = 0
    task_dirs = []
    for csv_path in results_dir.rglob("*.csv"):
        if "proof" in csv_path.parts or csv_path.name == "summary.csv":
            continue
        task_dirs.append(csv_path.parent)

    for task_dir in sorted(set(task_dirs)):
        if (task_dir / "proof.zip").exists():
            continue
        csv_path = latest_csv_in_dir(task_dir)
        if csv_path is None:
            continue
        task_name = task_dir.name
        cmd = [
            sys.executable,
            str(analyzer),
            "--csv", str(csv_path),
            "--output", str(task_dir),
            "--name", task_name,
        ]
        if no_plot:
            cmd.append("--no-plot")
        subprocess.run(cmd, check=True)
        if (task_dir / "proof.zip").exists():
            created += 1
    return created


def discover_proofs(results_dir):
    proofs = []
    for proof_zip in sorted(results_dir.rglob("proof.zip")):
        if "local_rank_ready" in proof_zip.parts:
            continue
        try:
            manifest = read_manifest_from_zip(proof_zip)
        except Exception as exc:
            print("warning: skip invalid proof %s: %s" % (proof_zip, exc))
            continue
        proofs.append((proof_zip, manifest))
    return proofs


def choose_best_per_task(proofs):
    best = {}
    for proof_zip, manifest in proofs:
        task_name = manifest.get("task_name", proof_zip.parent.name)
        score = float(manifest.get("score", 0.0))
        old = best.get(task_name)
        if old is None or score > float(old[1].get("score", 0.0)):
            best[task_name] = (proof_zip, manifest)
    return [best[key] for key in sorted(best)]


def build_package(results_dir, controller, output_dir, zip_name, select_best,
                  analyze_missing, no_plot):
    workspace_root = find_workspace_root()
    proof_tool = find_proof_tool(workspace_root)
    results_dir = Path(results_dir).resolve()
    output_dir = Path(output_dir).resolve()
    package_dir = output_dir / "local_rank_ready"
    zip_path = output_dir / zip_name

    if analyze_missing:
        created = ensure_missing_proofs(results_dir, workspace_root, no_plot)
        if created:
            print("created %d missing proof package(s)" % created)

    proofs = discover_proofs(results_dir)
    if select_best:
        proofs = choose_best_per_task(proofs)
    if not proofs:
        raise RuntimeError("no proof.zip files found under %s" % results_dir)

    if package_dir.exists():
        shutil.rmtree(package_dir)
    package_dir.mkdir(parents=True)
    controller_dir = package_dir / safe_name(controller)
    controller_dir.mkdir(parents=True)

    rows = []
    used_names = set()
    verified = 0
    for proof_zip, manifest in proofs:
        task_name = manifest.get("task_name", proof_zip.parent.name)
        dest_base = "%s_proof.zip" % safe_name(task_name)
        if dest_base in used_names:
            rel_parent = proof_zip.parent.relative_to(results_dir)
            dest_base = "%s_%s_proof.zip" % (
                safe_name(str(rel_parent)),
                safe_name(task_name),
            )
        used_names.add(dest_base)

        dest = controller_dir / dest_base
        shutil.copy2(proof_zip, dest)

        status = verify_proof_zip(proof_tool, dest)
        if status == "ok":
            verified += 1

        rows.append({
            "controller": controller,
            "source_group": str(proof_zip.parent.relative_to(results_dir)),
            "task": task_name,
            "score": manifest.get("score", ""),
            "grade": manifest.get("grade", ""),
            "phase_delay_seconds": manifest.get("phase_delay_seconds", ""),
            "scoring_mode": manifest.get("scoring_mode", "raw"),
            "proof_zip": str(dest.relative_to(output_dir)),
            "status": status,
        })

    summary_path = package_dir / "summary.csv"
    fields = [
        "controller",
        "source_group",
        "task",
        "score",
        "grade",
        "phase_delay_seconds",
        "scoring_mode",
        "proof_zip",
        "status",
    ]
    with summary_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    readme_path = package_dir / "README.txt"
    with readme_path.open("w", encoding="utf-8") as f:
        f.write("Local ranking submission package\n")
        f.write("Source results: %s\n" % results_dir)
        f.write("Controller: %s\n" % controller)
        f.write("Proof packages: %d\n" % len(rows))
        f.write("Verified locally: %d\n" % verified)
        f.write("Each proof zip contains result.csv, manifest.json, signature.txt.\n")

    if zip_path.exists():
        zip_path.unlink()
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(package_dir.rglob("*")):
            if path.is_file():
                zf.write(path, path.relative_to(output_dir))

    print("submission_dir=%s" % package_dir)
    print("submission_zip=%s" % zip_path)
    print("proof_count=%d" % len(rows))
    print("verified_count=%d" % verified)
    return zip_path


def main():
    parser = argparse.ArgumentParser(
        description="Generate local ranking submission package"
    )
    parser.add_argument("--results-dir", required=True,
                        help="Benchmark result directory to package")
    parser.add_argument("--controller", default="controller",
                        help="Controller name written to summary.csv")
    parser.add_argument("--output-dir",
                        help="Output directory; defaults to results-dir")
    parser.add_argument("--zip-name", default="local_rank_ready.zip")
    parser.add_argument("--select-best", action="store_true",
                        help="When repeated tasks exist, keep best score per task")
    parser.add_argument("--analyze-missing", action="store_true",
                        help="Run benchmark_analyzer for task dirs missing proof.zip")
    parser.add_argument("--no-plot", action="store_true",
                        help="Use --no-plot when --analyze-missing is enabled")
    args = parser.parse_args()

    output_dir = args.output_dir or args.results_dir
    build_package(
        args.results_dir,
        args.controller,
        output_dir,
        args.zip_name,
        args.select_best,
        args.analyze_missing,
        args.no_plot,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
