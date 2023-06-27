#ifndef SSC_H_
#define SSC_H_

#include "utility.h"
#include "patchwork.h"
#include "tictoc.h"

class SSC: public Utility{
public:
    int id;

    int range_num; 
    int sector_num;
    int azimuth_num;
    int bin_num;

    std::vector<PointAPRI> apri_vec;
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud_downSample;
    pcl::PointCloud<pcl::PointXYZI>::Ptr vox_cloud;  // voxel cloud -> intensity = voxel_idx
    std::unordered_map<int, Voxel> hash_cloud;
    std::unordered_map<int, Object> object_set;

    boost::shared_ptr<PatchWork<pcl::PointXYZI>> PatchworkGroundSeg;   // patchwork

    ~SSC();
    SSC() = delete;
    SSC(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
        TicToc("ssc initialization");

        allocateMemory();
        range_num = (int)std::ceil((max_range - min_range) / range_res);   // get ssc params
        sector_num = (int)std::ceil((max_angle - min_angle) / angle_res);
        azimuth_num = (int)std::ceil((max_azimuth - min_azimuth) / azimuth_res);
        bin_num = range_num * sector_num * azimuth_num;

        // extract ground
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudUse(new pcl::PointCloud<pcl::PointXYZI>());
        cloudUse = extractGroudByPatchWork(cloudIn_);

        // make apri
        apri_vec = makeApriVec(cloudUse);

        // make hash table


        // ROS_DEBUG();
    }

    void allocateMemory(){
        g_cloud_downSample.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr extractGroudByPatchWork(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
        double time_pw;
        pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        PatchworkGroundSeg->set_sensor(sensor_height);
        PatchworkGroundSeg->estimate_ground(*cloudIn_, *g_cloud, *ng_cloud, time_pw);
        cloudDownSample(g_cloud, g_cloud_downSample);
        return ng_cloud;
    }

    std::vector<PointAPRI> makeApriVec(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
        std::vector<PointAPRI> tmp_vec;
        for(size_t i =0; i < cloudIn_->points.size(); i++){
            pcl::PointXYZI pt = cloudIn_->points[i];
            float dis = pointDistance2d(pt);
            float angle = getPolarAngle(pt);
            float azimuth = getAzimuth(pt);
            if(dis < min_range || dis > max_range){
                continue;
            }
            if(angle < min_angle || angle > max_angle){
                continue;
            }
            if(azimuth < min_azimuth || azimuth > max_azimuth){
                continue;
            }

            PointAPRI apri;
            apri.x = pt.x;
            apri.y = pt.y;
            apri.z = pt.z;
            apri.range = dis;
            apri.angle = angle;
            apri.azimuth = azimuth;
            apri.intensity = pt.intensity;
            apri.range_idx = std::ceil((dis - min_range) / range_res) - 1;
            apri.sector_idx = std::ceil((angle - min_angle) / angle_res) - 1;
            apri.azimuth_idx = std::ceil((azimuth - min_azimuth) / azimuth_res) -1;
            apri.voxel_idx = apri.azimuth_idx * range_num * sector_num + apri.range_idx * sector_num + apri.sector_idx;
            if(apri.voxel_idx > bin_num){
                ROS_WARN("pt %d can't find its bin", (int)i);
                continue;
            }
            tmp_vec.emplace_back(apri);
        }
        return tmp_vec;
    }

    std::unordered_map<int, Voxel> makeHashCloud(const std::vector<PointAPRI>& apriIn_){
        std::unordered_map<int, Voxel>::iterator it_find;

        for(size_t i = 0; i < apriIn_.size(); i++){
            PointAPRI apri = apriIn_[i];
            if(it_find != hash_cloud.end()){
                it_find->second.ptIdx.emplace_back(i);
            }
            else{
                Voxel voxel;
                voxel.ptIdx.emplace_back(i);
                voxel.range_idx = apri.range_idx;
                voxel.sector_idx = apri.sector_idx;
                voxel.azimuth_idx = apri.azimuth_idx;
                voxel.voxCenter.intensity = apri.voxel_idx;
                hash_cloud.insert(std::make_pair(apri.voxel_idx, voxel));
            }
        }

        for(auto& vox : hash_cloud){
            float in_sum = 0.0;
            float center_x = 0.0;
            float center_y = 0.0;
            float center_z = 0.0;
            for(auto& k : vox.second.ptIdx){
                center_x += apriIn_[k].x;
                center_y += apriIn_[k].y;
                center_z += apriIn_[k].z;
                in_sum += apriIn_[k].intensity;
            }
            vox.second.voxCenter.x = center_x / vox.second.ptIdx.size();
            vox.second.voxCenter.y = center_y / vox.second.ptIdx.size();
            vox.second.voxCenter.z = center_z / vox.second.ptIdx.size();
            vox.second.intensity_av = in_sum / vox.second.ptIdx.size();

            float in_cov = 0.0;
            for(auto& k : vox.second.ptIdx){
                in_cov += std::pow((apriIn_[k].intensity - vox.second.intensity_av), 2);
            }
            vox.second.intensity_cov = in_cov / vox.second.ptIdx.size();
        }
    }
};

#endif