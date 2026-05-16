# Proof Materials Guide

## Best Proof Screenshot
Colored point cloud in Open3D or RViz:
- Green points = structurally nominal (within 3cm of baseline)
- Red points = flagged defects (>3cm deviation)
- Color gradient showing deviation magnitude

## Required Visual Proofs

| Proof | Description |
|-------|-------------|
| Point cloud screenshots | Colored defect visualization for 2-3 structures |
| Before/after ICP | Raw scan vs aligned scan side by side |
| Defect segmentation | Red/green colored output with legend |
| Deployment photo | SICK MultiScan mounted on structure/rig |
| Quantitative chart | Bar chart: accuracy before (62%) vs after (81%) |
| False positive comparison | Before/after FP rate visualization |

## Demo Video
Shows real-time point cloud loading → ICP alignment → defect highlighting across multiple structure scans with CSV export at end.
