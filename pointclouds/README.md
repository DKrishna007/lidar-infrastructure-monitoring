# Point Cloud Data

Store baseline and inspection scan PCD files here.

## Format
- .pcd or .ply format
- - Collected via SICK MultiScan 100
  - - 50k-200k points per scan
    - - Voxel downsampled to 2cm before ICP
     
      - ## Structure
      - baselines/ - Reference scans (do not overwrite)
      - scans/     - Periodic inspection scans
