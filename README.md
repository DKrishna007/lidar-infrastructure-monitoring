# LiDAR Infrastructure Monitoring

> Real-time structural health monitoring using SICK MultiScan 100, ROS 2, PCL, and KISS-ICP. Deployed across 10+ structures. Defect detection accuracy improved from 60-65% to 80-82%.
>
> **Researcher:** Krishna Digamarthi | University of Delaware
>
> ---
>
> ## Problem
> Manual structural inspection is slow, expensive, and inconsistent. Traditional methods require 10-14 hours per structure per cycle. This system automates defect detection using 3D LiDAR point cloud analysis with centimeter-level accuracy.
>
> ## Hardware
> - **Sensor:** SICK MultiScan 100 (360 degree coverage)
> - - **Compute:** ROS 2 workstation / edge device
>   - - **Stack:** ROS 2, PCL, KISS-ICP, Open3D
>    
>     - ## Results
>    
>     - | Metric | Value |
>     - |--------|-------|
>     - | Accuracy | **80-82%** (from 60-65%) |
> | False positive reduction | **25-35%** |
> | Inspection time saved | **8-12 hrs/cycle** |
> | Cost reduction | **40-55%** |
> | Structures deployed | **10+** |
>
> ## How to Run
>
> ```bash
> pip install -r requirements.txt
> python src/defect_detector.py
> python scripts/run_inspection.py --scans /data/scans --refs /data/references
> ```
>
> ## Proof Materials
>
> | Type | Location |
> |------|----------|
> | Architecture | docs/architecture.md |
> | Evaluation | docs/evaluation.md |
> | Sample results | results/defect_report_sample.csv |
> | Proof guide | docs/proof_guide.md |
>
> ## Limitations
> - Requires pre-collected baseline scan per structure
> - - ICP may fail on featureless planar surfaces
>   - - Outdoor deployment affected by wind vibration
>    
>     - ## Future Work
>     - - Automated baseline update scheduling
>       - - Severity classification (minor/moderate/critical)
>         - - Web dashboard for multi-structure monitoring
