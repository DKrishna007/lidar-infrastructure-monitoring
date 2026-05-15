#!/usr/bin/env python3
"""
LiDAR-Based Structural Defect Detector
Detects surface defects (cracks, spalling, displacement) using SICK MultiScan100 
point cloud data with PCL processing and KISS-ICP registration
"""
import numpy as np
import open3d as o3d
import logging
import time
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field
from pathlib import Path
from enum import Enum

logger = logging.getLogger(__name__)


class DefectType(Enum):
    """Classification of structural defects"""
    CRACK = "crack"
    SPALLING = "spalling"
    DEFORMATION = "deformation"
    DISPLACEMENT = "displacement"
    CORROSION = "corrosion"
    UNKNOWN = "unknown"


@dataclass
class DefectRegion:
    """Represents a detected structural defect"""
    defect_type: DefectType
    centroid: np.ndarray          # 3D centroid position
    bounding_box_min: np.ndarray  # 3D bounding box minimum
    bounding_box_max: np.ndarray  # 3D bounding box maximum
    area_m2: float                # Estimated surface area
    severity: float               # Severity score [0, 1]
    point_count: int              # Number of points in region
    surface_normal: np.ndarray    # Dominant surface normal
    roughness: float              # Surface roughness measure
    scan_timestamp: float
    scan_id: str = ""
    confidence: float = 0.0
    
    @property
    def severity_label(self) -> str:
        if self.severity < 0.3:
            return "low"
        elif self.severity < 0.7:
            return "medium"
        else:
            return "high"


class SICKMultiScanProcessor:
    """
    Processes point clouds from SICK MultiScan100 LiDAR for structural monitoring.
    
    The SICK MultiScan100 provides:
    - 16 scan layers, 360° horizontal
    - Range: 0.1 - 100m
    - Resolution: 0.33° horizontal, ~2.8° layer spacing
    - Point rate: 1.2 MHz (combined)
    
    Processing pipeline:
    1. Voxel downsampling (5cm voxels for structural analysis)
    2. Statistical outlier removal
    3. Normal estimation
    4. Surface segmentation (planar structures)
    5. Defect region extraction
    6. Defect classification
    """
    
    def __init__(self,
                 voxel_size: float = 0.02,       # 2cm voxels for detail
                 normal_radius: float = 0.1,      # 10cm normal estimation
                 outlier_nb: int = 20,            # Neighbors for outlier removal
                 outlier_std: float = 2.0,        # Std threshold for outliers
                 min_cluster_points: int = 50,
                 max_cluster_points: int = 100000,
                 crack_threshold: float = 0.015,   # 15mm depth = crack
                 spall_threshold: float = 0.050,   # 50mm depth = spalling
                 deform_threshold: float = 0.005): # 5mm = deformation
        self.voxel_size = voxel_size
        self.normal_radius = normal_radius
        self.outlier_nb = outlier_nb
        self.outlier_std = outlier_std
        self.min_cluster_points = min_cluster_points
        self.max_cluster_points = max_cluster_points
        self.crack_threshold = crack_threshold
        self.spall_threshold = spall_threshold
        self.deform_threshold = deform_threshold
        
        # Reference point cloud for change detection
        self.reference_cloud: Optional[o3d.geometry.PointCloud] = None
        self.reference_id: str = ""
        
        logger.info(f"SICKMultiScanProcessor initialized (voxel={voxel_size}m)")
    
    def preprocess(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """
        Preprocess raw LiDAR point cloud.
        Steps: voxel downsample → outlier removal → normal estimation
        """
        # Voxel downsampling
        pcd_down = pcd.voxel_down_sample(self.voxel_size)
        
        # Statistical outlier removal (remove noise points)
        pcd_clean, ind = pcd_down.remove_statistical_outlier(
            nb_neighbors=self.outlier_nb,
            std_ratio=self.outlier_std
        )
        
        # Estimate surface normals
        pcd_clean.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.normal_radius, max_nn=30
            )
        )
        pcd_clean.orient_normals_consistent_tangent_plane(100)
        
        return pcd_clean
    
    def extract_planes(self, 
                       pcd: o3d.geometry.PointCloud) -> List[Tuple[np.ndarray, o3d.geometry.PointCloud]]:
        """
        Extract dominant planar surfaces (walls, floors, columns) using RANSAC.
        Returns list of (plane_model, plane_inlier_cloud) tuples.
        """
        planes = []
        remaining = pcd
        
        for _ in range(10):  # Extract up to 10 planes
            if len(remaining.points) < self.min_cluster_points:
                break
            
            # RANSAC plane fitting
            plane_model, inliers = remaining.segment_plane(
                distance_threshold=0.02,  # 2cm inlier threshold
                ransac_n=3,
                num_iterations=1000
            )
            
            if len(inliers) < self.min_cluster_points:
                break
            
            plane_cloud = remaining.select_by_index(inliers)
            planes.append((plane_model, plane_cloud))
            
            # Remove inliers and continue
            remaining = remaining.select_by_index(inliers, invert=True)
        
        logger.debug(f"Extracted {len(planes)} planar surfaces")
        return planes
    
    def compute_deviation_map(self,
                              plane_cloud: o3d.geometry.PointCloud,
                              plane_model: np.ndarray) -> np.ndarray:
        """
        Compute point deviation from ideal plane surface.
        Large deviations indicate surface defects.
        
        Returns: deviation array (N,) in meters
        """
        points = np.asarray(plane_cloud.points)
        a, b, c, d = plane_model
        normal = np.array([a, b, c])
        
        # Signed distance from plane: (ax + by + cz + d) / sqrt(a²+b²+c²)
        distances = (points @ normal + d) / np.linalg.norm(normal)
        return distances
    
    def cluster_defects(self,
                        plane_cloud: o3d.geometry.PointCloud,
                        deviations: np.ndarray,
                        threshold: float) -> List[o3d.geometry.PointCloud]:
        """
        Find clusters of points exceeding deviation threshold.
        Uses DBSCAN clustering to group connected defect regions.
        """
        # Select outlier points
        outlier_mask = np.abs(deviations) > threshold
        outlier_indices = np.where(outlier_mask)[0]
        
        if len(outlier_indices) < self.min_cluster_points:
            return []
        
        outlier_cloud = plane_cloud.select_by_index(outlier_indices.tolist())
        
        # DBSCAN clustering
        labels = np.array(outlier_cloud.cluster_dbscan(
            eps=0.05,        # 5cm cluster radius
            min_points=10,   # Minimum 10 points per cluster
            print_progress=False
        ))
        
        clusters = []
        n_clusters = labels.max() + 1 if labels.max() >= 0 else 0
        
        for i in range(n_clusters):
            cluster_idx = np.where(labels == i)[0]
            if self.min_cluster_points <= len(cluster_idx) <= self.max_cluster_points:
                cluster_cloud = outlier_cloud.select_by_index(cluster_idx.tolist())
                clusters.append(cluster_cloud)
        
        return clusters
    
    def classify_defect(self,
                        cluster: o3d.geometry.PointCloud,
                        deviations_subset: np.ndarray) -> DefectType:
        """
        Classify defect type based on geometric features:
        - Depth, aspect ratio, roughness, normal variance
        """
        points = np.asarray(cluster.points)
        
        # Compute extent
        bbox = cluster.get_axis_aligned_bounding_box()
        extent = bbox.get_extent()
        
        depth = np.max(np.abs(deviations_subset))
        aspect = max(extent[0], extent[1]) / (min(extent[0], extent[1]) + 1e-6)
        
        # Roughness: mean distance to centroid projected on plane
        roughness = np.std(deviations_subset)
        
        # Decision logic based on geometric features
        if depth > self.spall_threshold and extent[0]*extent[1] > 0.04:
            return DefectType.SPALLING
        elif depth < self.crack_threshold and aspect > 5.0:
            return DefectType.CRACK
        elif roughness > 0.01 and depth > self.deform_threshold:
            return DefectType.DEFORMATION
        else:
            return DefectType.UNKNOWN
    
    def analyze_plane(self,
                      plane_cloud: o3d.geometry.PointCloud,
                      plane_model: np.ndarray,
                      scan_id: str = "",
                      timestamp: float = 0.0) -> List[DefectRegion]:
        """
        Full defect analysis of a single planar surface.
        Returns list of detected defect regions.
        """
        defects = []
        
        # Compute deviations from ideal plane
        deviations = self.compute_deviation_map(plane_cloud, plane_model)
        
        # Find defect clusters
        clusters = self.cluster_defects(plane_cloud, deviations, self.deform_threshold)
        
        points_array = np.asarray(plane_cloud.points)
        
        for cluster in clusters:
            cluster_pts = np.asarray(cluster.points)
            
            # Find deviation subset for this cluster
            dists = np.linalg.norm(
                points_array[:, np.newaxis, :] - cluster_pts[np.newaxis, :, :],
                axis=2
            )
            cluster_deviations = deviations[dists.min(axis=1) < 0.1]
            
            defect_type = self.classify_defect(cluster, cluster_deviations)
            
            centroid = cluster.get_center()
            bbox = cluster.get_axis_aligned_bounding_box()
            
            # Estimate surface area from bounding box
            extent = bbox.get_extent()
            area = extent[0] * extent[1]
            
            # Severity based on depth and area
            max_depth = np.max(np.abs(cluster_deviations)) if len(cluster_deviations) > 0 else 0
            severity = min(1.0, max_depth / 0.1 * 0.5 + area / 1.0 * 0.5)
            
            # Surface normal from normals in cluster
            normals = np.asarray(cluster.normals) if cluster.has_normals() else np.zeros((1, 3))
            avg_normal = normals.mean(axis=0) if len(normals) > 0 else np.array([0, 0, 1])
            
            roughness = float(np.std(cluster_deviations)) if len(cluster_deviations) > 0 else 0.0
            
            defect = DefectRegion(
                defect_type=defect_type,
                centroid=centroid,
                bounding_box_min=bbox.min_bound,
                bounding_box_max=bbox.max_bound,
                area_m2=area,
                severity=severity,
                point_count=len(cluster_pts),
                surface_normal=avg_normal,
                roughness=roughness,
                scan_timestamp=timestamp,
                scan_id=scan_id,
                confidence=0.75 + 0.25 * severity
            )
            defects.append(defect)
        
        return defects
    
    def process_scan(self,
                     pcd: o3d.geometry.PointCloud,
                     scan_id: str = "",
                     timestamp: float = 0.0) -> List[DefectRegion]:
        """
        Full processing pipeline: preprocess → plane extraction → defect detection
        """
        t0 = time.time()
        
        # Preprocess
        pcd_clean = self.preprocess(pcd)
        
        # Extract planes
        planes = self.extract_planes(pcd_clean)
        
        # Analyze each plane for defects
        all_defects = []
        for plane_model, plane_cloud in planes:
            defects = self.analyze_plane(plane_cloud, plane_model, scan_id, timestamp)
            all_defects.extend(defects)
        
        elapsed = (time.time() - t0) * 1000
        logger.info(f"Scan {scan_id}: {len(all_defects)} defects in {elapsed:.1f}ms "
                   f"(from {len(planes)} planes)")
        
        return all_defects
    
    def set_reference(self, pcd: o3d.geometry.PointCloud, ref_id: str):
        """Set reference (baseline) point cloud for change detection"""
        self.reference_cloud = self.preprocess(pcd)
        self.reference_id = ref_id
        logger.info(f"Reference set: {ref_id} ({len(self.reference_cloud.points)} pts)")
    
    def compute_change(self, 
                       current_pcd: o3d.geometry.PointCloud) -> Optional[np.ndarray]:
        """
        Compare current scan to reference using nearest-neighbor distances.
        Large distances indicate structural changes.
        """
        if self.reference_cloud is None:
            return None
        
        current_clean = self.preprocess(current_pcd)
        
        # Compute distances from current to reference
        dists = current_clean.compute_point_cloud_distance(self.reference_cloud)
        return np.asarray(dists)


if __name__ == '__main__':
    # Demo with synthetic point cloud
    processor = SICKMultiScanProcessor(voxel_size=0.02)
    
    # Generate synthetic wall surface with a crack
    n_pts = 10000
    xy = np.random.uniform(-1, 1, (n_pts, 2))
    z = np.zeros(n_pts)
    
    # Add crack: deep groove along x-axis near center
    crack_mask = (np.abs(xy[:, 1]) < 0.01) & (xy[:, 0] > -0.3) & (xy[:, 0] < 0.3)
    z[crack_mask] = -0.02  # 2cm deep crack
    
    points = np.column_stack([xy[:, 0], z, xy[:, 1]])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    defects = processor.process_scan(pcd, scan_id="demo_001", timestamp=time.time())
    
    for defect in defects:
        print(f"Defect: {defect.defect_type.value}, "
              f"severity={defect.severity_label}, "
              f"area={defect.area_m2:.3f}m², "
              f"pts={defect.point_count}")
