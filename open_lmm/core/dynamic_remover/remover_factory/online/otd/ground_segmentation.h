#ifndef GROUND_SEGMENTATION_H
#define GROUND_SEGMENTATION_H

#define PCL_NO_PRECOMPILE

#include <omp.h>

// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/io/pcd_io.h> 

namespace otd {

template <typename PointT>
class Grd_Seg {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Grd_Seg(double sensor_height, double tau_seeds, double tau_dis, bool kitti_en = false);
    ~Grd_Seg(){}

    void Run(typename pcl::PointCloud<PointT>::Ptr raw_ptr, typename pcl::PointCloud<PointT>::Ptr ground_ptr, 
      typename pcl::PointCloud<PointT>::Ptr nonground_ptr, const Eigen::Matrix4d pose);
    void GetTimeConsume();

   private:
    void SeparateLines(typename pcl::PointCloud<PointT>::Ptr raw_ptr);
    void GroundSegmentation(typename pcl::PointCloud<PointT>::Ptr raw_ptr, typename pcl::PointCloud<PointT>::Ptr ground_ptr, 
      typename pcl::PointCloud<PointT>::Ptr nonground_ptr, const Eigen::Matrix4d pose);

   private:
    std::vector<pcl::PointCloud<PointT>> pc_with_scan;
    std::vector<std::vector<bool>> ground_en_with_scan;

    // 这个量vector中每一维代表一条线，将每条线中所有的点都存在了一个map中。
    // map中first代表点的方位角[-180, 180]，second代表点；
    std::vector<std::map<float, PointT>> point_angle;

    float sensor_height_ = 1;
    float tau_seeds_ = 1.2;
    float tau_dis_ = 0.3;
    bool kitti_en_;
    
    // Time log
    double t_preprocess = 0.0, t_refine = 0.0, t_separatelines = 0.0, t_downsample = 0.0;
    double t_preprocess_sum = 0.0, t_refine_sum = 0.0, t_separatelines_sum = 0.0, t_downsample_sum = 0.0;
    int frame_sum = 0;
};

template <typename PointT>
Grd_Seg<PointT>::Grd_Seg(double sensor_height, double tau_seeds, double tau_dis, bool kitti_en) : 
    sensor_height_(-sensor_height), tau_seeds_(tau_seeds), tau_dis_(tau_dis), kitti_en_(kitti_en)
{}

template <typename PointT>
void Grd_Seg<PointT>::GetTimeConsume()
{
    double t_groundseg_sum = t_preprocess_sum + t_refine_sum + t_downsample_sum;
    printf("The average ground segmentation time is %lf\n", (t_groundseg_sum/frame_sum) * 1000);
}

//=====================================================================================//
//Ground Segmentation for KITTI//
template <typename PointT>
void Grd_Seg<PointT>::Run(typename pcl::PointCloud<PointT>::Ptr raw_ptr, typename pcl::PointCloud<PointT>::Ptr ground_ptr, 
  typename pcl::PointCloud<PointT>::Ptr nonground_ptr, const Eigen::Matrix4d pose)
{
    if(kitti_en_) SeparateLines(raw_ptr);

    GroundSegmentation(raw_ptr, ground_ptr, nonground_ptr, pose);

    t_separatelines_sum += t_separatelines;
    t_preprocess_sum += t_preprocess;
    t_refine_sum += t_refine;
    t_downsample_sum += t_downsample;
    frame_sum++;
}

template <typename PointT>
void Grd_Seg<PointT>::SeparateLines(typename pcl::PointCloud<PointT>::Ptr raw_ptr)
{
    int index[65]={0};

    int cloud_size = raw_ptr->size();
    int beam_count=0;
    float last_point_angle;// 序列中前一个点的方位角
    float back_point_angle;// 序列中前两个点的方位角
    // 这俩用来去除序列中的前几个点
    bool selected = true;
    int selectcount = 0;

    for (int i=0; i<cloud_size; i++)
    {
        // 计算方位角时用-x计算，是为了让方位角从π开始，π——π/2——0——(-π/2)——(-π)。
        float angle = atan2(raw_ptr->points[i].y , -raw_ptr->points[i].x ) * 180 / M_PI;   
        if (i==0) last_point_angle = angle, back_point_angle = angle; //handle the first point


        if(selected&&(selectcount<10)) {selectcount++;}
        else selected = false;

        // ~是取反操作符,对二进制来讲与!一样
        if (((angle-last_point_angle)>=90)&&(~selected)&&((angle-back_point_angle)>=90)) 
        {
        beam_count++ ;
        index[beam_count] = i;
        selected = true;
        selectcount = 0;
        }
        back_point_angle = last_point_angle;
        last_point_angle = angle;
    }

    if(beam_count<63)
    {
        for (int i=beam_count; i<64; i++)
        {
        index[i] = index[beam_count];
        }
        
        std::cout << "scans less than 64" << std::endl;  //beams fly !
    }
    else if(beam_count>=64)
    {
        std::cout << "gg, sort failed" << std::endl; // may fail
        return ;
    }

    index[64] = raw_ptr->size()-1;
    int scan_id = 63;

    pc_with_scan.clear();
    pc_with_scan.resize(64);
    ground_en_with_scan.clear();
    ground_en_with_scan.resize(64);
    point_angle.clear();
    point_angle.resize(64);

    for (int it = 0; it < 64; it++)
    {
        PointT point;
        if ( it < 63)
        {
            for (int i = index[it]; i < index[it+1]; i++)
            {
                // raw_ptr->points[i].ring = scan_id;
                pc_with_scan[scan_id].push_back(raw_ptr->points[i]);     
                ground_en_with_scan[scan_id].push_back(false);
                float angle = atan2(raw_ptr->points[i].y , -raw_ptr->points[i].x ) * 180 / M_PI;   
                point_angle[scan_id].insert(std::pair<float, PointT>(angle, raw_ptr->points[i]));
            }
        }
        else
        {
            for (int i = index[it]; i < cloud_size; i++)
            {
                // raw_ptr->points[i].ring = scan_id;
                pc_with_scan[scan_id].push_back(raw_ptr->points[i]);
                ground_en_with_scan[scan_id].push_back(false);
                float angle = atan2(raw_ptr->points[i].y , -raw_ptr->points[i].x ) * 180 / M_PI;   
                point_angle[scan_id].insert(std::pair<float, PointT>(angle, raw_ptr->points[i]));
            }
        }
        scan_id--;
    }

}

template <typename PointT>
void Grd_Seg<PointT>::GroundSegmentation(typename pcl::PointCloud<PointT>::Ptr raw_ptr, typename pcl::PointCloud<PointT>::Ptr ground_ptr, 
  typename pcl::PointCloud<PointT>::Ptr nonground_ptr, const Eigen::Matrix4d pose)
{
    pcl::PointCloud<PointT> cloud_nonground_init, cloud_ground_init;
    pcl::PointCloud<PointT> cloud_ground_seeds, cloud_ground_iter, cloud_nonground_iter;
    double t_preprocess_start = omp_get_wtime();

    if(kitti_en_)
    {
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        int find_num = 0;

        for (size_t i = 59; i < 64; i++)
        {
            cloud_nonground_init += pc_with_scan[i];
        }

        #pragma omp parallel for
        for (size_t i = 0; i < 58; i++)
        {
            std::map<float, PointT> pts_angle_temp = point_angle[i + 2];
            for (size_t j = 0; j < pc_with_scan[i].size(); j++)
            {
                PointT pt_temp = pc_with_scan[i].points[j];
                float pt_angle = atan2(pt_temp.y , -pt_temp.x ) * 180 / M_PI;
                auto iter = pts_angle_temp.lower_bound(pt_angle);
                if (iter != pts_angle_temp.end())
                {
                    PointT pt_next = iter->second;
                    diffX = pt_next.x - pt_temp.x;
                    diffY = pt_next.y - pt_temp.y;
                    diffZ = pt_next.z - pt_temp.z;
                    angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
                    if (abs(angle) <= 15) // 是地面点
                        ground_en_with_scan[i][j] = true;
                }
            }
        }
        for (size_t i = 0; i < 58; i++)
        {
            for (size_t j = 0; j < pc_with_scan[i].size(); j++)
            {
                PointT pt_temp = pc_with_scan[i].points[j];
                if (ground_en_with_scan[i][j])
                    cloud_ground_init.push_back(pt_temp);
                else
                    cloud_nonground_init.push_back(pt_temp);
            }
        }
    }
    else {
        cloud_ground_init = *raw_ptr;
    }
    // printf("The ground points size after preprocess is %ld\n", cloud_ground_init.points.size());
    t_preprocess = omp_get_wtime() - t_preprocess_start;

    double t_refine_start = omp_get_wtime();
    for(int i=0;i<cloud_ground_init.points.size();i++){
        if(cloud_ground_init.points[i].z > sensor_height_ - tau_seeds_)
        {
            if (cloud_ground_init.points[i].z < sensor_height_ + tau_seeds_)
                cloud_ground_seeds.points.push_back(cloud_ground_init.points[i]);
            else
                cloud_nonground_init.points.push_back(cloud_ground_init.points[i]);
        }
    }

    std::sort(cloud_ground_seeds.points.begin(),cloud_ground_seeds.points.end(), [] (PointT a, PointT b){ return a.z<b.z; });

    for(int i=0;i<cloud_ground_seeds.points.size();i++){
        if(cloud_ground_seeds.points[i].z > sensor_height_ - tau_seeds_/3)
        {
            if (cloud_ground_seeds.points[i].z < sensor_height_ + tau_seeds_/3)
                cloud_ground_iter.points.push_back(cloud_ground_seeds.points[i]);
            else
                break;
        }
    }
    // printf("The ground points size after seeds  is %ld\n", cloud_ground_iter.points.size());

    Eigen::MatrixXf points(cloud_ground_seeds.points.size(),3);
    int j =0;
    for(auto p:cloud_ground_seeds.points){
        points.row(j++)<<p.x,p.y,p.z;
    }

    for(int i=0;i<3;i++){
        Eigen::Matrix3f cov;
        Eigen::Vector4f pc_mean;
        pcl::computeMeanAndCovarianceMatrix(cloud_ground_iter, cov, pc_mean);
        // Singular Value Decomposition: SVD
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
        // use the least singular vector as normal
        Eigen:: MatrixXf normal_ = (svd.matrixU().col(2));
        // mean ground seeds value
        Eigen::Vector3f seeds_mean = pc_mean.head<3>();

        // according to normal.T*[x,y,z] = -d
        float d_ = -(normal_.transpose()*seeds_mean)(0,0);
        // set distance threhold to `th_dist - d`
        float tau_disd_ = tau_dis_ - d_;

        cloud_ground_iter.clear();
        cloud_nonground_iter.clear();
        //pointcloud to matrix

        // ground plane model
        Eigen::VectorXf result = points*normal_;
        // threshold filter
        for(int r=0;r<result.rows();r++){
            if(result[r]<tau_disd_){
                cloud_ground_iter.points.push_back(cloud_ground_seeds[r]);
            }else{
                cloud_nonground_iter.points.push_back(cloud_ground_seeds[r]);
            }
        }
    }
    // printf("The ground points size after refine is %ld\n", cloud_ground_iter.points.size());
    cloud_nonground_iter += cloud_nonground_init;

    t_refine = omp_get_wtime() - t_refine_start;

    for (size_t i = 0; i < cloud_ground_iter.size(); i++)
    {
        Eigen::Vector4d ptVec0, ptVec1;
        ptVec0 <<cloud_ground_iter.points[i].x, cloud_ground_iter.points[i].y, cloud_ground_iter.points[i].z, 1;
        ptVec1 = pose * ptVec0;

        cloud_ground_iter.points[i].x = ptVec1[0];
        cloud_ground_iter.points[i].y = ptVec1[1];
        cloud_ground_iter.points[i].z = ptVec1[2];
        ground_ptr->push_back(cloud_ground_iter.points[i]);
    }

    for (size_t i = 0; i < cloud_nonground_iter.size(); i++)
    {
        Eigen::Vector4d ptVec0, ptVec1;
        ptVec0 <<cloud_nonground_iter.points[i].x, cloud_nonground_iter.points[i].y, cloud_nonground_iter.points[i].z, 1;
        ptVec1 = pose * ptVec0;

        cloud_nonground_iter.points[i].x = ptVec1[0];
        cloud_nonground_iter.points[i].y = ptVec1[1];
        cloud_nonground_iter.points[i].z = ptVec1[2];
        nonground_ptr->push_back(cloud_nonground_iter.points[i]);
    }
}


}  // namespace otd

#endif  // GROUND_SEGMENTATION_H