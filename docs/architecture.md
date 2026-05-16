# Architecture

## Pipeline

```
SICK MultiScan 100 (10+ structures deployed)
│
│ ROS 2 PointCloud2
▼
┌─────────────────────────────────────────┐
│ STAGE 1: Point Cloud Acquisition        │
│ ROS 2 driver | 360° coverage            │
│ Voxel downsampling (2cm grid)           │
└──────────────┬──────────────────────────┘
               │
               ┌──────────────▼──────────────────────────┐
               │ STAGE 2: KISS-ICP Alignment             │
               │ Current scan → Reference scan           │
               │ Point-to-plane ICP                      │
               │ fitness score validation                │
               └──────────────┬──────────────────────────┘
                              │
                              ┌──────────────▼──────────────────────────┐
                              │ STAGE 3: Defect Detection               │
                              │ Point-to-point distance computation     │
                              │ Threshold: 3cm flags potential defect   │
                              │ Color coding: red=defect, green=ok      │
                              └──────────────┬──────────────────────────┘
                                             │
                                             ┌──────────────▼──────────────────────────┐
                                             │ STAGE 4: Report Generation              │
                                             │ CSV: structure_id, deviation, ratio     │
                                             │ Visualization: colored point cloud      │
                                             │ Flagging: ratio > 2% → alert           │
                                             └─────────────────────────────────────────┘
                                             ```

                                             ## Deployment Stats
                                             - Structures monitored: 10+
                                             - Detection accuracy: 80–82% (up from 60–65%)
                                             - False positive reduction: 25–35%
                                             - Processing time saved: 8–12 hrs/inspection cycle
                                             - Cost reduction: 40–55%
