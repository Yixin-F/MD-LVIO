## MD-LVIO
### This repo belongs to my own postgraduate research project, so the whole project will be opensource until I graduate from SEU !!
- Dynamic-Aware LIO with GPU Accelaration (half-done)
- Graph-Based Relocalization (done)
- Multi-Session Map Fusion (done)
- Object-Level Detection and Update (Todo)

#### Segmented Curved-Voxel Occupancy Descriptor for Dynamic-aware LiDAR Odometry and Mapping (TGRS)
The main contributions are as follows:
1) We propose a novel egocentric descriptor called [Segmented Curved-Voxel Occupancy Descriptor (SCV-OD)](https://github.com/Yixin-F/DR-Using-SCV-OD), based on which a dynamic-aware and LiDAR-only
SLAM framework is built for mobile sensing with various sensor types in unknown and complex environments.
2) We propose a multi-stage object segmentation method assisted with LiDAR intensity and geometric features. This method is tightly coupled with a lightweight object
tracking approach, allowing for the effective removal of high dynamic objects and the refinement of imperfect objects through curved-voxel occupancy detection.
3) We modify VGICP as the pipeline for the SCV-OD, efficiently estimating initial motions at 10 Hz and update LiDAR poses with considering dynamic removal at 15 -20 Hz.
Consequently, a high-quality static instance map in the global coordinate system is generated.
4) The proposed method is quantitatively and qualitatively evaluated on the KITTI dataset and the custom dataset.
Finally, we demonstrate that our proposed approach outperforms the state-of-the-art studies in static mapping
