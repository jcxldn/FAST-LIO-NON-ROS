#ifndef LASER_MAPPING_H_
#define LASER_MAPPING_H_

#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ikd-Tree/ikd_Tree.h>

#include "common_lib.h"
#include "msgs.h"
#include "use-ikfom.hpp"
#include "preprocess.h"
#include "IMU_Processing.hpp"

#define CONFIG_FILE_PATH std::string("../config/horizon.json")
#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)

extern PointCloudXYZI::Ptr feats_down_body;
extern PointCloudXYZI::Ptr feats_down_world;
extern PointCloudXYZI::Ptr normvec;
extern PointCloudXYZI::Ptr laserCloudOri;
extern PointCloudXYZI::Ptr corr_normvect;

extern double solve_time;
extern double match_time;

extern double res_mean_last;
extern double total_residual;

extern bool extrinsic_est_en;

extern float res_last[100000];

extern int effct_feat_num;
extern int feats_down_size;
extern bool point_selected_surf[100000];

extern vector<PointVector> Nearest_Points;
extern KD_TREE<PointType> ikdtree;

class LaserMapping
{
public:
    LaserMapping(/* args */);
    ~LaserMapping();

    void livox_pcl_cbk(const custom_messages::CstMsgConstPtr &msg);
    void imu_cbk(const custom_messages::ImuConstPtr &msg_in);
    void run(custom_messages::OdomMsgPtr &msg_in);

private:
    void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s);
    void pointBodyToWorld(PointType const *const pi, PointType *const po);
    void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
    void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
    void points_cache_collect();
    void lasermap_fov_segment();
    bool sync_packages(MeasureGroup &meas);
    void map_incremental();
    void update_odometry(custom_messages::OdomMsgPtr &msg_in);
    static void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
    {
        double match_start = omp_get_wtime();
        laserCloudOri->clear();
        corr_normvect->clear();
        total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (int i = 0; i < feats_down_size; i++)
        {
            PointType &point_body = feats_down_body->points[i];
            PointType &point_world = feats_down_world->points[i];

            /* transform to world frame */
            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
            point_world.x = p_global(0);
            point_world.y = p_global(1);
            point_world.z = p_global(2);
            point_world.intensity = point_body.intensity;

            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

            auto &points_near = Nearest_Points[i];

            if (ekfom_data.converge)
            {
                /** Find the closest surfaces in the map **/
                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                                    : true;
            }

            if (!point_selected_surf[i])
                continue;

            VF(4)
            pabcd;
            point_selected_surf[i] = false;
            if (esti_plane(pabcd, points_near, 0.1f))
            {
                float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                if (s > 0.9)
                {
                    point_selected_surf[i] = true;
                    normvec->points[i].x = pabcd(0);
                    normvec->points[i].y = pabcd(1);
                    normvec->points[i].z = pabcd(2);
                    normvec->points[i].intensity = pd2;
                    res_last[i] = abs(pd2);
                }
            }
        }

        effct_feat_num = 0;

        for (int i = 0; i < feats_down_size; i++)
        {
            if (point_selected_surf[i])
            {
                laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                corr_normvect->points[effct_feat_num] = normvec->points[i];
                total_residual += res_last[i];
                effct_feat_num++;
            }
        }

        if (effct_feat_num < 1)
        {
            ekfom_data.valid = false;
            //   ROS_WARN("No Effective Points! \n");
            return;
        }

        res_mean_last = total_residual / effct_feat_num;
        match_time += omp_get_wtime() - match_start;
        double solve_start_ = omp_get_wtime();

        /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
        ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); // 23
        ekfom_data.h.resize(effct_feat_num);

        for (int i = 0; i < effct_feat_num; i++)
        {
            const PointType &laser_p = laserCloudOri->points[i];
            V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            M3D point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);

            /*** get the normal vector of closest surface/corner ***/
            const PointType &norm_p = corr_normvect->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            /*** calculate the Measuremnt Jacobian matrix H ***/
            V3D C(s.rot.conjugate() * norm_vec);
            V3D A(point_crossmat * C);
            if (extrinsic_est_en)
            {
                V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); // s.rot.conjugate()*norm_vec);
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            }
            else
            {
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }

            /*** Measuremnt: distance to the closest surface/corner ***/
            ekfom_data.h(i) = -norm_p.intensity;
        }
        solve_time += omp_get_wtime() - solve_start_;
    }

    template <typename T>
    void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po);

    template <typename T>
    void set_posestamp(T &out);

private:
    static const int MAXN = 720000;
    double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
    double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
    double solve_const_H_time = 0;
    int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
    bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, path_en = true;

    double msr_freq = 0.0, main_freq = 0.0;
    double timediff_lidar_wrt_imu = 0.0;

    float DET_RANGE = 300.0f;
    const float MOV_THRESHOLD = 1.5f;
    double time_diff_lidar_to_imu = 0.0;

    mutex mtx_buffer;
    condition_variable sig_buffer;

    string root_dir = ROOT_DIR;
    string map_file_path, lid_topic, imu_topic;

    double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
    double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
    double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
    int time_log_counter = 0, scan_count = 0, publish_count = 0;
    int iterCount = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
    bool lidar_pushed, flg_first_scan = true, flg_EKF_inited;
    bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

    vector<vector<int>> pointSearchInd_surf;
    vector<BoxPointType> cub_needrm;
    vector<double> extrinT;
    vector<double> extrinR;
    deque<double> time_buffer;
    deque<PointCloudXYZI::Ptr> lidar_buffer;
    deque<custom_messages::ImuConstPtr> imu_buffer;

    PointCloudXYZI::Ptr featsFromMap;
    PointCloudXYZI::Ptr feats_undistort;
    PointCloudXYZI::Ptr _featsArray;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    V3F XAxisPoint_body;
    V3F XAxisPoint_world;
    V3D euler_cur;
    V3D position_last;
    V3D Lidar_T_wrt_IMU;
    M3D Lidar_R_wrt_IMU;

    /*** EKF inputs and output ***/
    MeasureGroup Measures;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    state_ikfom state_point;
    vect3 pos_lid;

    custom_messages::Odometry odomAftMapped;
    custom_messages::Quaternion geoQuat;
    custom_messages::PoseStamped msg_body_pose;

    shared_ptr<Preprocess> p_pre;
    shared_ptr<ImuProcess> p_imu;

    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
};

#endif