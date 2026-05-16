"""
Batch Inspection Runner
Run defect detection on all structure scans in a directory.
Usage: python scripts/run_inspection.py --scans /data/scans --refs /data/references
"""
import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from defect_detector import InfrastructureMonitor


def run_batch(scans_dir: str, refs_dir: str, output_csv: str = "results/defect_report.csv"):
      monitor = InfrastructureMonitor(
                voxel_size=0.02,
                icp_threshold=0.05,
                defect_threshold=0.03
      )

    scan_files = sorted(glob.glob(os.path.join(scans_dir, "*.pcd")) +
                                                glob.glob(os.path.join(scans_dir, "*.ply")))

    print(f"[Inspector] Found {len(scan_files)} scan files in {scans_dir}")

    for scan_path in scan_files:
              structure_id = os.path.splitext(os.path.basename(scan_path))[0]
              ref_path = os.path.join(refs_dir, os.path.basename(scan_path))

        if not os.path.exists(ref_path):
                      print(f"[Inspector] WARNING: No reference for {structure_id}, skipping.")
                      continue

        print(f"\n[Inspector] Processing: {structure_id}")
        current = monitor.load_pcd(scan_path)
        reference = monitor.load_pcd(ref_path)

        transform = monitor.align_icp(current, reference)
        current.transform(transform)

        annotated, result = monitor.detect_defects(current, reference, structure_id)
        status = "FLAGGED" if result["flagged"] else "OK"
        print(f"[Inspector] {structure_id}: {status} | "
                            f"defect_ratio={result['defect_ratio_pct']}% | "
                            f"max_dev={result['max_deviation_m']}m")

    monitor.save_report(output_csv)
    print(f"\n[Inspector] Complete. Report: {output_csv}")


if __name__ == "__main__":
      p = argparse.ArgumentParser()
      p.add_argument("--scans", required=True, help="Directory of current scan PCD files")
      p.add_argument("--refs",  required=True, help="Directory of reference/baseline PCD files")
      p.add_argument("--output", default="results/defect_report.csv", help="Output CSV path")
      args = p.parse_args()
      run_batch(args.scans, args.refs, args.output)
