#include "multiAgentMapping/distributed_mapping/lidarIrisDescriptor.hpp"

lidar_iris_descriptor::lidar_iris_descriptor (
        int rows = 80,
        int columns = 360,
        int n_scan = 128,
        double distance_threshold = 0.32,
        int exclude_recent_frame_num = 30,
        int match_mode = 2,
        int candidates_num = 10,
        int nscale = 4,
        int min_wave_length = 18,
        float mult = 1.6,
        float sigma_on_f = 0.75,
        int robot_num = 1,
        int id = 0):
    rows_(rows), // 80 in the original paper
    columns_(columns), // 360 in the original paper
    n_scan_(n_scan), // lidar sensor lines
    distance_threshold_(distance_threshold), // 0.10-0.45 is ok.
    // 0.10-0.15 is very fine (rare false-alarms);
    // 0.15-0.35 is good but ICP fitness score check should be required for robustness;
    // 0.35-0.45 is choice for using with robust outlier rejection.
    exclude_recent_frame_num_(exclude_recent_frame_num), // simply just keyframe gap
    match_mode_(match_mode), // 0 for detecting same direction
    // 1 for detecting opposite direction
    // 2 for detecting both same and opposite direction
    candidates_num_(candidates_num), // 6-10 is enough
    nscale_(nscale), // 4 in the original paper
    min_wave_length_(min_wave_length), // 18 in the original paper
    mult_(mult), // 1.6 in the original paper
    sigma_on_f_(sigma_on_f), // 0.75 in the original paper
    robot_num_(robot_num), // number of robot in robotic swarm
    id_(id) // this robot id
{
    for (int i = 0; i < robot_num_; i++) {
        unordered_map < int, lidar_iris_descriptor::featureDesc > base_feature_desc;
        unordered_map < int, Eigen::MatrixXf > base_matrix;
        unordered_map < int, int > base_int;
        iris_features.push_back(base_feature_desc);
        iris_rowkeys.push_back(base_matrix);
        indexes_maps.push_back(base_int);
    }
}

lidar_iris_descriptor::~lidar_iris_descriptor(){}

std::pair<Eigen::VectorXf, cv::Mat1b> lidar_iris_descriptor::getIris(
    const pcl::PointCloud<pcl::PointXYZI> &cloud
){
    cv::Mat1b iris_image = cv::Mat1b::zeros(rows_, columns_);
    Eigen::MatrixXf iris_row_key_matrix = Eigen::MatrixXf::Zero(rows_, columns_);
    // VLS 128 lidar logic
    if(n_scan_ == 128){ 
        for(auto p : cloud.points){
            // compute XY-plane distance
            float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);

            // vertical angle remapping: [-25 to +15] -> [0, 40]
            float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 25;

            // horizontal angle remapping [-180, +180] -> [0, 360]
            float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;

            // discrtize into bins
            int Q_dis = std::min(std::max((int)floor(dis), 0), (rows_ - 1));
            int Q_arc = std::min(std::max((int)floor(arc / 5.0f), 0), 7);  // 8 bins, 5° each
            int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), (columns_ - 1));

            // update iris image with bit-wise encoding
            iris_image.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);

            // Update the row key matrix to store the maximum height (z-value)
            if(iris_row_key_matrix(Q_dis, Q_yaw) < p.data[2]){
                iris_row_key_matrix(Q_dis, Q_yaw) = p.data[2];
            }
        }
    } else {
        std::cout << "[LiDAR Iris Descriptor] Error : Only n_scan = 128 supported" << std::endl;
    }

    // extract rowkey
    Eigen::VectorXf rowkey = Eigen::VectorXf::Zero(rows_);
    for (int i = 0; i < iris_row_key_matrix.rows(); i++){
        Eigen::VectorXf curr_row = iris_row_key_matrix.row(i);
        rowkey(i) = curr_row.mean();
    }

    return std::make_pair(rowkey, iris_image);
}

inline cv::Mat lidar_iris_descriptor::circularRowShift(
    const cv::Mat &src,
    int shift_m_rows
){
    if(shift_m_rows == 0){
        return src.clone();
    }
    shift_m_rows %= src.rows;
    int m = shift_m_rows > 0 ? shift_m_rows : src.rows + shift_m_rows;
    cv::Mat dst(src.size(), src.type());
    src(cv::Range(src.rows - m, src.rows), cv::Range::all()).copyTo(dst(cv::Range(0, m), cv::Range::all()));
	src(cv::Range(0, src.rows - m), cv::Range::all()).copyTo(dst(cv::Range(m, src.rows), cv::Range::all()));
	return dst;
}

inline cv::Mat lidar_iris_descriptor::circularColShift(
    const cv::Mat &src,
    int shift_n_cols
){
    if(shift_n_cols == 0){
        return src.clone();
    }
    shift_n_cols %= src.cols;
    int n = shift_n_cols > 0 ? shift_n_cols : src.cols + shift_n_cols;
    cv::Mat dst(src.size(), src.type());
	src(cv::Range::all(), cv::Range(src.cols - n, src.cols)).copyTo(dst(cv::Range::all(), cv::Range(0, n)));
	src(cv::Range::all(), cv::Range(0, src.cols - n)).copyTo(dst(cv::Range::all(), cv::Range(n, src.cols)));
	return dst;
}

cv::Mat lidar_iris_descriptor::circularShift(
	const cv::Mat &src,
	int shift_m_rows,
	int shift_n_cols)
{
	return circularColShift(circularRowShift(src, shift_m_rows), shift_n_cols);
}