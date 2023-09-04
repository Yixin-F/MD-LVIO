## MD-LVIO
### This repo belongs to my own postgraduate research project, so the whole project will be opensource until I graduate from SEU !!
- Dynamic-Aware LIO with GPU Accelaration (Half-done)
- Graph-Based Relocalization (Done)
- Multi-Session Map Fusion (Done)
- Object-Level Detection and Update (Todo)
- Application (Done)

### 0. Our sensing platform
<img src="https://github.com/Yixin-F/MD-LVIO/blob/main/doc/platform.png" width="50%">

### 1. Dynamic-Aware LIO with GPU Accelaration 
The main contributions are as follows:
1) We propose a novel egocentric descriptor called [Segmented Curved-Voxel Occupancy Descriptor (SCV-OD)](https://github.com/Yixin-F/DR-Using-SCV-OD), based on which a dynamic-aware and LiDAR-only
SLAM framework is built for mobile sensing with various sensor types in unknown and complex environments.
2) We propose a multi-stage object segmentation method assisted with LiDAR intensity and geometric features. This method is tightly coupled with a lightweight object
tracking approach, allowing for the effective removal of high dynamic objects and the refinement of imperfect objects through curved-voxel occupancy detection.
3) We modify VGICP as the pipeline for the SCV-OD, efficiently estimating initial motions at 10 Hz and update LiDAR poses with considering dynamic removal at 15 -20 Hz.
Consequently, a high-quality static instance map in the global coordinate system is generated.
4) The proposed method is quantitatively and qualitatively evaluated on the KITTI dataset and the custom dataset.
Finally, we demonstrate that our proposed approach outperforms the state-of-the-art studies in static mapping

#### 1) System Overview
<centre>
<img src="https://github.com/Yixin-F/MD-LVIO/blob/main/doc/framework.png" width="80%">
</centre>

#### 2) Evaluation on Kitti
<centre>
<img src="https://github.com/Yixin-F/MD-LVIO/blob/main/doc/kitti.png" width="60%">
</centre>

#### 3) Evaluation on Our Custom Dataset
<centre>
<img src="https://github.com/Yixin-F/MD-LVIO/blob/main/doc/custom.png" width="60%">
</centre>

### 2. Graph-Based Relocalization
Unlike most of the existing methods that focus on extracting local, global, and statistical features of raw point clouds, our method aims at the semantic level that can be superior in terms of robustness to environmental changes. Inspired
by the perspective of humans, who recognize scenes through identifying semantic objects and capturing their relations, [this paper](https://github.com/kxhit/SG_PR) presents a novel semantic graph based approach for place recognition. First, we propose a novel semantic graph representation for the point cloud scenes by reserving the semantic and topological information of the raw point cloud. Thus, place recognition is modeled as a graph matching problem. Then we design a fast and effective graph similarity network to compute the similarity. Exhaustive evaluations on the KITTI dataset show that our approach is robust to the occlusion as well as viewpoint changes and outperforms the state-of-theart methods with a large margin.
#### 1) Framework
<centre>
<img src="https://github.com/Yixin-F/MD-LVIO/blob/main/doc/graph.png" width="70%">
</centre>

#### 2) Evaluation on Kitti
<img src="https://github.com/Yixin-F/MD-LVIO/blob/main/doc/lpr.png" width="60%"> <img src="https://github.com/Yixin-F/MD-LVIO/blob/main/doc/lpkitti.png" width="60%"> 

### 3. Multi-Session Map Fusion
Long-term 3D map management is a fundamental capability required by a robot to reliably navigate in the non-stationary real-world. [This paper](https://github.com/Yixin-F/LT-mapper_fyx) develops open-source, modular, and readily available LiDAR-based lifelong mapping
for urban sites. This is achieved by dividing the problem into successive subproblems: multi-session SLAM (MSS), high/low dynamic change detection, and positive/negative change management. The proposed method leverages MSS and minimizes
potential trajectory error; thus, a manual or good initial alignment is not required for change detection. Our change management scheme preserves efficacy in both memory and computation costs, providing automatic object segregation from a large-scale point cloud map.


