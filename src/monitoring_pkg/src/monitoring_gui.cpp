#ifndef INTEGRATED_ARUCO_SYSTEM_HPP
#define INTEGRATED_ARUCO_SYSTEM_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rcl_interfaces/srv/set_parameters.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QScrollArea>
#include <QtCore/QTimer>
#include <QtCore/QThread>
#include <QtGui/QImage>
#include <QtGui/QPixmap>

#include <QtGui/QPainter>           // ← 추가 필요
#include <QtGui/QPen>               // ← 추가 필요
#include <QtGui/QBrush>             // ← 추가 필요
#include <QtGui/QColor>             // ← 추가 필요
#include <QtGui/QCloseEvent>        // ← 추가 필요
#include <QtGui/QPaintEvent>        // ← 추가 필요


#include <memory>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

// ArUco 관련 함수들
namespace aruco_utils {
    
inline double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
}

} // namespace aruco_utils

// 로봇 상태 클래스
class RobotState {
public:
    RobotState() : x(0.0), y(0.0), yaw(0.0), vx(0.0), vy(0.0), vyaw(0.0),
                   last_x(0.0), last_y(0.0), last_yaw(0.0), 
                   initialized(false), last_time(std::chrono::steady_clock::now()) {}
    
    double x, y, yaw;
    double vx, vy, vyaw;
    double last_x, last_y, last_yaw;
    bool initialized;
    std::chrono::steady_clock::time_point last_time;
    std::chrono::steady_clock::time_point last_seen;
    
    void update_state(double x_raw, double y_raw, double yaw_raw, 
                     double position_alpha, double velocity_alpha, double yaw_alpha);
};

// ArUco 검출기 클래스
class ArucoDetector {
public:
    struct MarkerInfo {
        int id;
        std::pair<double, double> robot_xy;
        double yaw;
        cv::Point2f text_pos;
        std::vector<cv::Point2f> corners;
    };
    
    ArucoDetector(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, double marker_length = 0.05);
    
    std::vector<MarkerInfo> detect_markers(const cv::Mat& frame);
    
private:
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double marker_length_;
    cv::Mat homography_matrix_;
    
    std::pair<double, double> transform_point(const cv::Point2f& pt);
    void setup_homography();
};

// ROS 통합 스레드 클래스 (Qt 스레드 상속)
class IntegratedRosThread : public QThread {
    Q_OBJECT
    
public:
    explicit IntegratedRosThread(std::shared_ptr<rclcpp::Node> node, QObject* parent = nullptr);
    ~IntegratedRosThread();
    
    void set_pid_param(const std::string& name, double value);
    void set_goal_mover_param(const std::string& name, double value);
    void stop_processing();
    
signals:
    void state_update(QString state);
    void angle_update(double angle);
    void dist_update(double distance);
    void img_update(QImage image);
    void robot_states_update(QVariantMap robot_states);
    void performance_update(QVariantMap performance);
    
protected:
    void run() override;
    
private slots:
    void process_frame();
    
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<ArucoDetector> detector_;
    cv::VideoCapture cap_;
    
    // ROS Publishers and Subscribers
    std::unordered_map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers_;
    std::unordered_map<int, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_publishers_;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dist_sub_;
    
    // Parameter service clients
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr pid_param_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr goal_mover_param_client_;
    
    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Robot states
    std::unordered_map<int, std::unique_ptr<RobotState>> robot_states_;
    
    // Filter parameters
    double velocity_alpha_;
    double position_alpha_;
    double yaw_alpha_;
    
    // Control flags
    std::atomic<bool> running_;
    
    // Performance tracking
    int frame_count_;
    std::chrono::steady_clock::time_point last_fps_time_;
    
    void setup_aruco();
    void setup_ros_interfaces();
    void update_robot_state(int marker_id, double x, double y, double yaw);
    nav_msgs::msg::Odometry create_odometry_message(int marker_id);
    void set_parameter(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client,
                      const std::string& name, double value, const std::string& node_name);
};

// 맵 위젯 클래스
class MapWidget : public QWidget {
    Q_OBJECT
    
public:
    explicit MapWidget(const std::vector<std::vector<int>>& custom_map, QWidget* parent = nullptr);
    
public slots:
    void update_robot_positions(const QVariantMap& robot_states);
    
protected:
    void paintEvent(QPaintEvent* event) override;
    
private:
    std::vector<std::vector<int>> map_data_;
    QVariantMap robot_positions_;
    
    void draw_map(QPainter& painter);
    void draw_robots(QPainter& painter);
};

// 메인 GUI 윈도우 클래스
class IntegratedMainWindow : public QMainWindow {
    Q_OBJECT
    
public:
    explicit IntegratedMainWindow(IntegratedRosThread* ros_thread, 
                                 std::shared_ptr<rclcpp::Node> node, 
                                 QWidget* parent = nullptr);
    ~IntegratedMainWindow();
    
protected:
    void closeEvent(QCloseEvent* event) override;
    
private slots:
    void update_camera(const QImage& image);
    void update_robot_states(const QVariantMap& robot_states);
    void update_performance(const QVariantMap& performance);
    void update_fps_display();
    void emergency_stop();
    void system_reset();
    void save_parameters();
    void load_parameters();
    
private:
    IntegratedRosThread* ros_thread_;
    std::shared_ptr<rclcpp::Node> node_;
    
    // GUI Components
    QWidget* central_widget_;
    QGridLayout* main_layout_;
    
    // Map section
    QGroupBox* map_group_;
    MapWidget* map_widget_;
    QLabel* map_info_label_;
    
    // Camera section
    QGroupBox* camera_group_;
    QLabel* camera_label_;
    QLabel* cam_status_label_;
    QLabel* frame_count_label_;
    QLabel* detection_label_;
    
    // Control section
    QGroupBox* control_group_;
    QTabWidget* param_tabs_;
    std::unordered_map<std::string, QDoubleSpinBox*> pid_spinboxes_;
    std::unordered_map<std::string, QDoubleSpinBox*> goal_mover_spinboxes_;
    std::unordered_map<std::string, QDoubleSpinBox*> aruco_spinboxes_;
    
    // Status section
    QGroupBox* status_group_;
    QLabel* state_label_;
    QLabel* angle_label_;
    QLabel* distance_label_;
    QLabel* fps_label_;
    QLabel* ros_status_label_;
    QLabel* performance_label_;
    QScrollArea* robot_details_scroll_;
    QWidget* robot_details_widget_;
    QVBoxLayout* robot_details_layout_;
    
    // Control buttons
    QPushButton* emergency_btn_;
    QPushButton* reset_btn_;
    QPushButton* save_btn_;
    QPushButton* load_btn_;
    
    // Timers
    QTimer* fps_timer_;
    
    // Counters
    int frame_count_;
    int fps_counter_;
    
    void setup_ui();
    void setup_map_section();
    void setup_camera_section();
    void setup_control_section();
    void setup_status_section();
    void setup_pid_tab();
    void setup_goal_mover_tab();
    void setup_aruco_tab();
    void setup_control_buttons();
    void connect_ros_signals();
    void add_preset_buttons(QGridLayout* layout, int start_row);
    void apply_preset(const std::unordered_map<std::string, double>& preset_params);
};

// 메인 애플리케이션 클래스
class IntegratedArucoApp {
public:
    IntegratedArucoApp(int argc, char** argv);
    ~IntegratedArucoApp();
    
    int run();
    
private:
    std::unique_ptr<QApplication> qt_app_;
    std::shared_ptr<rclcpp::Node> ros_node_;
    std::unique_ptr<IntegratedRosThread> ros_thread_;
    std::unique_ptr<IntegratedMainWindow> main_window_;
    
    bool initialize_ros();
    bool initialize_qt();
    void cleanup();
};

#endif // INTEGRATED_ARUCO_SYSTEM_HPP

// =============================================================================
// 구현부 (implementation)
// =============================================================================

// RobotState 구현
void RobotState::update_state(double x_raw, double y_raw, double yaw_raw,
                             double position_alpha, double velocity_alpha, double yaw_alpha) {
    auto current_time = std::chrono::steady_clock::now();
    last_seen = current_time;
    
    if (!initialized) {
        x = x_raw;
        y = y_raw;
        yaw = yaw_raw;
        last_x = x_raw;
        last_y = y_raw;
        last_yaw = yaw_raw;
        last_time = current_time;
        initialized = true;
        return;
    }
    
    auto dt = std::chrono::duration<double>(current_time - last_time).count();
    if (dt <= 0) return;
    
    // Low pass filter 적용
    double x_filtered = position_alpha * x + (1 - position_alpha) * x_raw;
    double y_filtered = position_alpha * y + (1 - position_alpha) * y_raw;
    
    double yaw_diff = aruco_utils::normalize_angle(yaw_raw - yaw);
    double yaw_filtered = aruco_utils::normalize_angle(yaw + (1 - yaw_alpha) * yaw_diff);
    
    // 속도 계산
    double dx = x_filtered - last_x;
    double dy = y_filtered - last_y;
    double dyaw = aruco_utils::normalize_angle(yaw_filtered - last_yaw);
    
    double vx_new = dx / dt;
    double vy_new = dy / dt;
    double vyaw_new = dyaw / dt;
    
    // 속도 필터링
    vx = velocity_alpha * vx + (1 - velocity_alpha) * vx_new;
    vy = velocity_alpha * vy + (1 - velocity_alpha) * vy_new;
    vyaw = velocity_alpha * vyaw + (1 - velocity_alpha) * vyaw_new;
    
    // 상태 업데이트
    last_x = x;
    last_y = y;
    last_yaw = yaw;
    
    x = x_filtered;
    y = y_filtered;
    yaw = yaw_filtered;
    last_time = current_time;
}

// ArucoDetector 구현
ArucoDetector::ArucoDetector(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, double marker_length)
    : camera_matrix_(camera_matrix), dist_coeffs_(dist_coeffs), marker_length_(marker_length) {
    
    aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    aruco_params_ = cv::aruco::DetectorParameters::create();
    setup_homography();
}

void ArucoDetector::setup_homography() {
    // 호모그래피 매트릭스 설정
    std::vector<cv::Point2f> src_pts = {
        cv::Point2f(-1.420f, 0.097f),
        cv::Point2f(-1.528f, -0.830f),
        cv::Point2f(0.459f, 0.076f),
        cv::Point2f(0.410f, -0.845f)
    };
    
    std::vector<cv::Point2f> dst_pts = {
        cv::Point2f(0.05f, 0.05f),
        cv::Point2f(0.05f, 0.95f),
        cv::Point2f(1.95f, 0.05f),
        cv::Point2f(1.95f, 0.95f)
    };
    
    homography_matrix_ = cv::getPerspectiveTransform(src_pts, dst_pts);
}

std::pair<double, double> ArucoDetector::transform_point(const cv::Point2f& pt) {
    std::vector<cv::Point2f> src_pt = {pt};
    std::vector<cv::Point2f> dst_pt;
    cv::perspectiveTransform(src_pt, dst_pt, homography_matrix_);
    return std::make_pair(static_cast<double>(dst_pt[0].x), static_cast<double>(dst_pt[0].y));
}

std::vector<ArucoDetector::MarkerInfo> ArucoDetector::detect_markers(const cv::Mat& frame) {
    std::vector<MarkerInfo> detected;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    
    cv::aruco::detectMarkers(frame, aruco_dict_, corners, ids, aruco_params_);
    
    if (!ids.empty()) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);
        
        for (size_t i = 0; i < ids.size(); ++i) {
            MarkerInfo marker;
            marker.id = ids[i];
            
            // 위치 변환
            cv::Point2f world_pt(tvecs[i][0], tvecs[i][1]);
            auto transformed = transform_point(world_pt);
            marker.robot_xy = std::make_pair(std::round(transformed.first * 100) / 100.0,
                                           std::round(transformed.second * 100) / 100.0);
            
            // 회전 계산
            cv::Mat rot_mat;
            cv::Rodrigues(rvecs[i], rot_mat);
            double yaw = -(std::atan2(rot_mat.at<double>(1, 0), rot_mat.at<double>(0, 0)) - M_PI / 2);
            marker.yaw = aruco_utils::normalize_angle(yaw);
            
            // 텍스트 위치 계산
            cv::Point2f center(0, 0);
            for (const auto& corner : corners[i]) {
                center += corner;
            }
            center *= (1.0f / corners[i].size());
            marker.text_pos = center;
            
            // 코너 저장
            marker.corners = corners[i];
            
            detected.push_back(marker);
        }
    }
    
    return detected;
}

// IntegratedRosThread 구현
IntegratedRosThread::IntegratedRosThread(std::shared_ptr<rclcpp::Node> node, QObject* parent)
    : QThread(parent), node_(node), running_(true), frame_count_(0),
      velocity_alpha_(0.3), position_alpha_(0.7), yaw_alpha_(0.6) {
    
    setup_aruco();
    setup_ros_interfaces();
}

IntegratedRosThread::~IntegratedRosThread() {
    stop_processing();
}

void IntegratedRosThread::setup_aruco() {
    try {
        // 카메라 캘리브레이션 데이터 로드
        std::string base_path = "/home/addinnedu/Rotus_team/roscamp-repo-3/Ros_Files/src/monitoring_pkg/config/";
        cv::Mat camera_matrix, dist_coeffs;
        
        // OpenCV의 FileStorage를 사용하여 numpy 파일 대신 xml/yml 파일 사용
        cv::FileStorage fs_camera(base_path + "camera_matrix.xml", cv::FileStorage::READ);
        cv::FileStorage fs_dist(base_path + "dist_coeffs.xml", cv::FileStorage::READ);
        
        if (fs_camera.isOpened() && fs_dist.isOpened()) {
            fs_camera["camera_matrix"] >> camera_matrix;
            fs_dist["dist_coeffs"] >> dist_coeffs;
            
            detector_ = std::make_unique<ArucoDetector>(camera_matrix, dist_coeffs);
            
            // 카메라 초기화
            cap_.open("/dev/video_cam", cv::CAP_V4L2);
            cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1200);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 600);
            cap_.set(cv::CAP_PROP_FPS, 30);
            
            RCLCPP_INFO(node_->get_logger(), "ArUco detector initialized successfully");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load camera calibration files");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "ArUco setup failed: %s", e.what());
    }
}

void IntegratedRosThread::setup_ros_interfaces() {
    // Subscribers
    state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/state", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            emit state_update(QString::fromStdString(msg->data));
        });
    
    angle_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/angle_error", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
            emit angle_update(msg->data);
        });
    
    dist_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/distance_error", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
            emit dist_update(msg->data);
        });
    
    // Parameter service clients
    pid_param_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("/pid_controller/set_parameters");
    goal_mover_param_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("/goal_mover/set_parameters");
    
    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

void IntegratedRosThread::run() {
    RCLCPP_INFO(node_->get_logger(), "Integrated ROS Thread started");
    
    while (rclcpp::ok() && running_) {
        process_frame();
        rclcpp::spin_some(node_);
        QThread::msleep(33); // ~30 FPS
    }
}

void IntegratedRosThread::process_frame() {
    if (!cap_.isOpened() || !detector_) {
        return;
    }
    
    cv::Mat frame;
    if (!cap_.read(frame)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to read camera frame");
        return;
    }
    
    auto start_time = std::chrono::steady_clock::now();
    
    // ArUco 마커 검출
    auto detected_markers = detector_->detect_markers(frame);
    
    // 검출된 마커들 처리
    for (const auto& marker : detected_markers) {
        int marker_id = marker.id;
        double x = marker.robot_xy.first;
        double y = marker.robot_xy.second;
        double yaw = marker.yaw;
        
        // 프레임에 마커 정보 그리기
        std::vector<cv::Point> corners_int;
        for (const auto& corner : marker.corners) {
            corners_int.push_back(cv::Point(static_cast<int>(corner.x), static_cast<int>(corner.y)));
        }
        cv::polylines(frame, corners_int, true, cv::Scalar(0, 255, 0), 2);
        
        std::string text = "ID:" + std::to_string(marker_id) + 
                          " (" + std::to_string(x) + "," + std::to_string(y) + ") " +
                          std::to_string(yaw * 180.0 / M_PI) + "°";
        cv::putText(frame, text, cv::Point(marker.text_pos.x - 50, marker.text_pos.y - 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
        
        // 로봇 상태 업데이트
        update_robot_state(marker_id, x, y, yaw);
        
        // Publishers 생성 (처음 감지될 때)
        if (pose_publishers_.find(marker_id) == pose_publishers_.end()) {
            std::string pose_topic = "/robot" + std::to_string(marker_id) + "/pose";
            std::string odom_topic = "/odom_" + std::to_string(marker_id);
            
            pose_publishers_[marker_id] = node_->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
            odom_publishers_[marker_id] = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
        }
        
        // 메시지 발행
        auto& state = robot_states_[marker_id];
        if (state && state->initialized) {
            // PoseStamped 발행
            auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
            pose->header.stamp = node_->get_clock()->now();
            pose->header.frame_id = "map";
            pose->pose.position.x = state->x;
            pose->pose.position.y = state->y;
            pose->pose.position.z = 0.0;
            pose->pose.orientation = aruco_utils::yaw_to_quaternion(state->yaw);
            
            pose_publishers_[marker_id]->publish(*pose);
            
            // Odometry 발행
            auto odom = create_odometry_message(marker_id);
            if (odom.header.frame_id != "") {
                odom_publishers_[marker_id]->publish(odom);
            }
        }
    }
    
    // GUI로 이미지 전송 (BGR -> RGB 변환)
    cv::Mat rgb_frame;
    cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);
    QImage qimg(rgb_frame.data, rgb_frame.cols, rgb_frame.rows, rgb_frame.step, QImage::Format_RGB888);
    emit img_update(qimg);
    
    // 로봇 상태 정보 GUI로 전송
    QVariantMap robot_info;
    auto current_time = std::chrono::steady_clock::now();
    for (const auto& [robot_id, state] : robot_states_) {
        if (state && state->initialized) {
            auto time_diff = std::chrono::duration<double>(current_time - state->last_seen).count();
            if (time_diff < 1.0) {  // 1초 이내 검출
                QVariantMap robot_data;
                robot_data["x"] = state->x;
                robot_data["y"] = state->y;
                robot_data["yaw"] = state->yaw;
                robot_data["vx"] = state->vx;
                robot_data["vy"] = state->vy;
                robot_data["vyaw"] = state->vyaw;
                robot_info[QString::number(robot_id)] = robot_data;
            }
        }
    }
    emit robot_states_update(robot_info);
    
    // 성능 정보 전송
    auto end_time = std::chrono::steady_clock::now();
    auto total_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    if (!detected_markers.empty()) {
        QVariantMap perf_info;
        perf_info["total_time"] = total_time_ms;
        perf_info["marker_count"] = static_cast<int>(detected_markers.size());
        perf_info["fps"] = total_time_ms > 0 ? 1000.0 / total_time_ms : 0.0;
        emit performance_update(perf_info);
    }
    
    frame_count_++;
}

void IntegratedRosThread::update_robot_state(int marker_id, double x, double y, double yaw) {
    if (robot_states_.find(marker_id) == robot_states_.end()) {
        robot_states_[marker_id] = std::make_unique<RobotState>();
    }
    
    robot_states_[marker_id]->update_state(x, y, yaw, position_alpha_, velocity_alpha_, yaw_alpha_);
}

nav_msgs::msg::Odometry IntegratedRosThread::create_odometry_message(int marker_id) {
    nav_msgs::msg::Odometry odom;
    
    auto it = robot_states_.find(marker_id);
    if (it == robot_states_.end() || !it->second || !it->second->initialized) {
        return odom; // empty message
    }
    
    auto& state = it->second;
    
    odom.header.stamp = node_->get_clock()->now();
    odom.header.frame_id = "odom_" + std::to_string(marker_id);
    odom.child_frame_id = "base_link_" + std::to_string(marker_id);
    
    odom.pose.pose.position.x = state->x;
    odom.pose.pose.position.y = state->y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = aruco_utils::yaw_to_quaternion(state->yaw);
    
    // 속도를 body frame으로 변환
    double cos_yaw = std::cos(state->yaw);
    double sin_yaw = std::sin(state->yaw);
    double vx_body = cos_yaw * state->vx + sin_yaw * state->vy;
    double vy_body = -sin_yaw * state->vx + cos_yaw * state->vy;
    
    odom.twist.twist.linear.x = vx_body;
    odom.twist.twist.linear.y = vy_body;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = state->vyaw;
    
    // 공분산 설정
    std::array<double, 36> pose_cov = {
        0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.02
    };
    
    std::array<double, 36> twist_cov = {
        0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.05
    };
    
    std::copy(pose_cov.begin(), pose_cov.end(), odom.pose.covariance.begin());
    std::copy(twist_cov.begin(), twist_cov.end(), odom.twist.covariance.begin());
    
    return odom;
}

void IntegratedRosThread::set_pid_param(const std::string& name, double value) {
    set_parameter(pid_param_client_, name, value, "PID Controller");
}

void IntegratedRosThread::set_goal_mover_param(const std::string& name, double value) {
    set_parameter(goal_mover_param_client_, name, value, "Goal Mover");
}

void IntegratedRosThread::set_parameter(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client,
                                       const std::string& name, double value, const std::string& node_name) {
    if (!client->wait_for_service(std::chrono::milliseconds(100))) {
        return;
    }
    
    try {
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter param;
        param.name = name;
        param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param.value.double_value = value;
        request->parameters.push_back(param);
        
        client->async_send_request(request);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Set %s parameter error: %s", node_name.c_str(), e.what());
    }
}

void IntegratedRosThread::stop_processing() {
    running_ = false;
    if (cap_.isOpened()) {
        cap_.release();
        RCLCPP_INFO(node_->get_logger(), "Camera released");
    }
}

// MapWidget 구현
MapWidget::MapWidget(const std::vector<std::vector<int>>& custom_map, QWidget* parent)
    : QWidget(parent), map_data_(custom_map) {
    setFixedSize(700, 400);
    setStyleSheet("border: 2px solid blue; background-color: white;");
}

void MapWidget::paintEvent(QPaintEvent* event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    draw_map(painter);
    draw_robots(painter);
}

void MapWidget::draw_map(QPainter& painter) {
    // 맵 그리기
    int cell_width = width() / 22;  // 22 columns
    int cell_height = height() / 12; // 12 rows
    
    for (int row = 0; row < map_data_.size(); ++row) {
        for (int col = 0; col < map_data_[row].size(); ++col) {
            QRect cell_rect(col * cell_width, row * cell_height, cell_width, cell_height);
            
            if (map_data_[row][col] == 1) {
                painter.fillRect(cell_rect, QColor(0, 0, 0)); // 장애물 (검은색)
            } else {
                painter.fillRect(cell_rect, QColor(255, 255, 255)); // 빈 공간 (흰색)
            }
        }
    }
    
    // 격자 그리기
    painter.setPen(QPen(QColor(200, 200, 200), 1));
    for (int i = 0; i <= 22; ++i) {
        painter.drawLine(i * cell_width, 0, i * cell_width, height());
    }
    for (int i = 0; i <= 12; ++i) {
        painter.drawLine(0, i * cell_height, width(), i * cell_height);
    }
}

void MapWidget::draw_robots(QPainter& painter) {
    QStringList colors = {"red", "blue", "green", "orange", "purple", "cyan"};
    
    for (auto it = robot_positions_.begin(); it != robot_positions_.end(); ++it) {
        int robot_id = it.key().toInt();
        QVariantMap pos = it.value().toMap();
        
        double x = pos["x"].toDouble();
        double y = pos["y"].toDouble();
        double yaw = pos["yaw"].toDouble();
        
        // 좌표 변환 (맵 좌표계 -> 위젯 좌표계)
        int pixel_x = static_cast<int>(((x + 0.1) / 2.2) * width());   // x 오프셋 보정
        int pixel_y = static_cast<int>((1.0 - (y + 0.1) / 1.2) * height());  // y 오프셋 보정
        
        // 로봇 색상
        QColor color(colors[robot_id % colors.size()]);
        
        // 로봇 위치 (원)
        painter.setBrush(QBrush(color));
        painter.setPen(QPen(QColor(0, 0, 0), 2));
        painter.drawEllipse(pixel_x - 8, pixel_y - 8, 16, 16);
        
        // 방향 화살표
        int arrow_length = 20;
        int end_x = pixel_x + static_cast<int>(arrow_length * std::cos(yaw));
        int end_y = pixel_y - static_cast<int>(arrow_length * std::sin(yaw)); // y축 반전
        
        painter.setPen(QPen(color, 3));
        painter.drawLine(pixel_x, pixel_y, end_x, end_y);
        
        // 로봇 ID 텍스트
        painter.setPen(QPen(QColor(255, 255, 255)));
        painter.drawText(pixel_x + 12, pixel_y - 5, QString("R%1").arg(robot_id));
    }
}

void MapWidget::update_robot_positions(const QVariantMap& robot_states) {
    robot_positions_ = robot_states;
    update();
}

// IntegratedMainWindow 구현
IntegratedMainWindow::IntegratedMainWindow(IntegratedRosThread* ros_thread,
                                          std::shared_ptr<rclcpp::Node> node,
                                          QWidget* parent)
    : QMainWindow(parent), ros_thread_(ros_thread), node_(node), frame_count_(0), fps_counter_(0) {
    
    setWindowTitle("통합 ArUco GUI 모니터링 시스템");
    resize(1600, 1200);
    
    setup_ui();
    connect_ros_signals();
    
    // FPS 계산용 타이머
    fps_timer_ = new QTimer(this);
    connect(fps_timer_, &QTimer::timeout, this, &IntegratedMainWindow::update_fps_display);
    fps_timer_->start(1000);
}

IntegratedMainWindow::~IntegratedMainWindow() {
    // 소멸자에서 정리 작업은 closeEvent에서 처리
}

void IntegratedMainWindow::setup_ui() {
    central_widget_ = new QWidget;
    setCentralWidget(central_widget_);
    
    main_layout_ = new QGridLayout(central_widget_);
    main_layout_->setSpacing(10);
    
    setup_map_section();
    setup_camera_section();
    setup_control_section();
    setup_status_section();
    
    // 그리드 레이아웃 비율 설정
    main_layout_->setRowStretch(0, 1);
    main_layout_->setRowStretch(1, 1);
    main_layout_->setColumnStretch(0, 1);
    main_layout_->setColumnStretch(1, 1);
}

void IntegratedMainWindow::setup_map_section() {
    map_group_ = new QGroupBox("1분면: 맵 + 로봇 위치");
    QVBoxLayout* map_layout = new QVBoxLayout();
    
    // 커스텀 맵 데이터
    std::vector<std::vector<int>> custom_map = {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    };
    
    map_widget_ = new MapWidget(custom_map);
    map_layout->addWidget(map_widget_);
    
    // 맵 정보 표시
    QHBoxLayout* map_info_layout = new QHBoxLayout();
    map_info_label_ = new QLabel("맵 크기: 2.0m x 1.0m | 활성 로봇: 0");
    map_info_layout->addWidget(map_info_label_);
    map_layout->addLayout(map_info_layout);
    
    map_group_->setLayout(map_layout);
    main_layout_->addWidget(map_group_, 0, 0);
}

void IntegratedMainWindow::setup_camera_section() {
    camera_group_ = new QGroupBox("2분면: Live Camera + ArUco Detection");
    QVBoxLayout* cam_layout = new QVBoxLayout();
    
    camera_label_ = new QLabel();
    camera_label_->setFixedSize(700, 400);
    camera_label_->setStyleSheet("border: 2px solid green; background-color: black; color: white; font-size: 14px;");
    camera_label_->setAlignment(Qt::AlignCenter);
    camera_label_->setText("ArUco 카메라 연결 대기 중...");
    cam_layout->addWidget(camera_label_, 0, Qt::AlignCenter);
    
    // 카메라 상태 정보
    QHBoxLayout* cam_status_layout = new QHBoxLayout();
    cam_status_label_ = new QLabel("Status: Initializing...");
    frame_count_label_ = new QLabel("Frames: 0");
    detection_label_ = new QLabel("Detections: 0");
    cam_status_layout->addWidget(cam_status_label_);
    cam_status_layout->addWidget(frame_count_label_);
    cam_status_layout->addWidget(detection_label_);
    cam_layout->addLayout(cam_status_layout);
    
    camera_group_->setLayout(cam_layout);
    main_layout_->addWidget(camera_group_, 0, 1);
}

void IntegratedMainWindow::setup_control_section() {
    control_group_ = new QGroupBox("3분면: 제어 파라미터");
    QVBoxLayout* control_layout = new QVBoxLayout();
    
    param_tabs_ = new QTabWidget();
    
    setup_pid_tab();
    setup_goal_mover_tab();
    setup_aruco_tab();
    
    control_layout->addWidget(param_tabs_);
    control_group_->setLayout(control_layout);
    main_layout_->addWidget(control_group_, 1, 0);
}

void IntegratedMainWindow::setup_pid_tab() {
    QWidget* pid_widget = new QWidget();
    QGridLayout* pid_layout = new QGridLayout();
    
    std::vector<std::tuple<QString, std::string, double>> pid_params = {
        {"Angular P", "angular_P", 1.0},
        {"Angular I", "angular_I", 0.0},
        {"Angular D", "angular_D", 0.0},
        {"Linear P", "linear_P", 1.0},
        {"Linear I", "linear_I", 0.0},
        {"Linear D", "linear_D", 0.0}
    };
    
    for (int i = 0; i < pid_params.size(); ++i) {
        auto [label_text, param_name, default_value] = pid_params[i];
        
        pid_layout->addWidget(new QLabel(label_text), i, 0);
        
        QDoubleSpinBox* spinbox = new QDoubleSpinBox();
        spinbox->setRange(-10.0, 10.0);
        spinbox->setDecimals(3);
        spinbox->setSingleStep(0.001);
        spinbox->setValue(default_value);
        
        connect(spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                [this, param_name](double value) {
                    ros_thread_->set_pid_param(param_name, value);
                });
        
        pid_layout->addWidget(spinbox, i, 1);
        pid_spinboxes_[param_name] = spinbox;
        
        // 현재 값 표시
        QLabel* value_label = new QLabel(QString::number(default_value));
        pid_layout->addWidget(value_label, i, 2);
        
        connect(spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                [value_label](double value) {
                    value_label->setText(QString::number(value, 'f', 3));
                });
    }
    
    pid_widget->setLayout(pid_layout);
    param_tabs_->addTab(pid_widget, "PID Control");
}

void IntegratedMainWindow::setup_goal_mover_tab() {
    QWidget* goal_widget = new QWidget();
    QGridLayout* goal_layout = new QGridLayout();
    
    std::vector<std::tuple<QString, std::string, double, double, double, double>> goal_params = {
        {"Linear Gain (k_lin)", "k_lin", 0.3, 0.0, 5.0, 0.01},
        {"Angular Gain (k_ang)", "k_ang", 0.1, 0.0, 2.0, 0.01},
        {"Min Linear Speed", "min_linear_speed", 0.55, 0.0, 2.0, 0.05},
        {"Min Angular Speed", "min_angular_speed", 0.55, 0.0, 2.0, 0.05},
        {"Angle Tolerance (deg)", "angle_tolerance_deg", 16.0, 1.0, 90.0, 1.0},
        {"Position Tolerance (m)", "pos_tolerance", 0.03, 0.01, 1.0, 0.01}
    };
    
    for (int i = 0; i < goal_params.size(); ++i) {
        auto [label_text, param_name, default_value, min_val, max_val, step] = goal_params[i];
        
        goal_layout->addWidget(new QLabel(label_text), i, 0);
        
        QDoubleSpinBox* spinbox = new QDoubleSpinBox();
        spinbox->setRange(min_val, max_val);
        spinbox->setDecimals(3);
        spinbox->setSingleStep(step);
        spinbox->setValue(default_value);
        
        connect(spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                [this, param_name](double value) {
                    ros_thread_->set_goal_mover_param(param_name, value);
                });
        
        goal_layout->addWidget(spinbox, i, 1);
        goal_mover_spinboxes_[param_name] = spinbox;
        
        // 현재 값 표시
        QLabel* value_label = new QLabel(QString::number(default_value));
        goal_layout->addWidget(value_label, i, 2);
        
        connect(spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                [value_label](double value) {
                    value_label->setText(QString::number(value, 'f', 3));
                });
    }
    
    // 프리셋 버튼들 추가
    add_preset_buttons(goal_layout, goal_params.size());
    
    goal_widget->setLayout(goal_layout);
    param_tabs_->addTab(goal_widget, "Goal Mover");
}

void IntegratedMainWindow::setup_aruco_tab() {
    QWidget* aruco_widget = new QWidget();
    QGridLayout* aruco_layout = new QGridLayout();
    
    std::vector<std::tuple<QString, std::string, double, double, double, double>> filter_params = {
        {"Position Alpha", "position_alpha", 0.7, 0.0, 1.0, 0.1},
        {"Velocity Alpha", "velocity_alpha", 0.3, 0.0, 1.0, 0.1},
        {"Yaw Alpha", "yaw_alpha", 0.6, 0.0, 1.0, 0.1}
    };
    
    for (int i = 0; i < filter_params.size(); ++i) {
        auto [label_text, param_name, default_value, min_val, max_val, step] = filter_params[i];
        
        aruco_layout->addWidget(new QLabel(label_text), i, 0);
        
        QDoubleSpinBox* spinbox = new QDoubleSpinBox();
        spinbox->setRange(min_val, max_val);
        spinbox->setDecimals(2);
        spinbox->setSingleStep(step);
        spinbox->setValue(default_value);
        
        // ArUco 필터 값 변경 시 ROS 스레드의 값도 업데이트
        connect(spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                [this, param_name](double value) {
                    // TODO: Update ArUco filter parameters in ros_thread_
                });
        
        aruco_layout->addWidget(spinbox, i, 1);
        aruco_spinboxes_[param_name] = spinbox;
        
        QLabel* value_label = new QLabel(QString::number(default_value));
        aruco_layout->addWidget(value_label, i, 2);
        
        connect(spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                [value_label](double value) {
                    value_label->setText(QString::number(value, 'f', 2));
                });
    }
    
    // ArUco 상태 정보
    QGroupBox* aruco_info_group = new QGroupBox("ArUco 상태");
    QVBoxLayout* aruco_info_layout = new QVBoxLayout();
    
    QLabel* aruco_status_label = new QLabel("검출 상태: 대기 중");
    QLabel* marker_count_label = new QLabel("감지된 마커: 0");
    QLabel* detection_fps_label = new QLabel("검출 FPS: 0");
    
    aruco_info_layout->addWidget(aruco_status_label);
    aruco_info_layout->addWidget(marker_count_label);
    aruco_info_layout->addWidget(detection_fps_label);
    aruco_info_group->setLayout(aruco_info_layout);
    
    aruco_layout->addWidget(aruco_info_group, filter_params.size(), 0, 1, 3);
    
    aruco_widget->setLayout(aruco_layout);
    param_tabs_->addTab(aruco_widget, "ArUco Settings");
}

void IntegratedMainWindow::setup_status_section() {
    status_group_ = new QGroupBox("4분면: 로봇 상태 모니터링");
    QVBoxLayout* status_layout = new QVBoxLayout();
    
    // 로봇 상태 표시
    QGroupBox* robot_status_group = new QGroupBox("로봇 상태");
    QVBoxLayout* robot_status_layout = new QVBoxLayout();
    
    // 기본 상태 정보
    state_label_ = new QLabel("전체 상태: 대기 중");
    angle_label_ = new QLabel("각도 오차: 0.00");
    distance_label_ = new QLabel("거리 오차: 0.00");
    robot_status_layout->addWidget(state_label_);
    robot_status_layout->addWidget(angle_label_);
    robot_status_layout->addWidget(distance_label_);
    
    // 개별 로봇 상태 표시 (스크롤 영역)
    robot_details_scroll_ = new QScrollArea();
    robot_details_widget_ = new QWidget();
    robot_details_layout_ = new QVBoxLayout(robot_details_widget_);
    robot_details_scroll_->setWidget(robot_details_widget_);
    robot_details_scroll_->setWidgetResizable(true);
    robot_details_scroll_->setMaximumHeight(200);
    robot_status_layout->addWidget(robot_details_scroll_);
    
    robot_status_group->setLayout(robot_status_layout);
    status_layout->addWidget(robot_status_group);
    
    // 시스템 정보 표시
    QGroupBox* system_info_group = new QGroupBox("시스템 정보");
    QVBoxLayout* system_info_layout = new QVBoxLayout();
    fps_label_ = new QLabel("카메라 FPS: 0");
    ros_status_label_ = new QLabel("ROS 상태: 연결됨");
    performance_label_ = new QLabel("처리 성능: 대기 중");
    system_info_layout->addWidget(fps_label_);
    system_info_layout->addWidget(ros_status_label_);
    system_info_layout->addWidget(performance_label_);
    system_info_group->setLayout(system_info_layout);
    status_layout->addWidget(system_info_group);
    
    // 제어 버튼들
    setup_control_buttons();
    QGroupBox* control_buttons_group = new QGroupBox("제어 버튼");
    QGridLayout* control_buttons_layout = new QGridLayout();
    
    // 긴급 정지 버튼
    emergency_btn_ = new QPushButton("긴급 정지");
    emergency_btn_->setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 14px;");
    emergency_btn_->setFixedHeight(40);
    connect(emergency_btn_, &QPushButton::clicked, this, &IntegratedMainWindow::emergency_stop);
    control_buttons_layout->addWidget(emergency_btn_, 0, 0);
    
    // 시스템 리셋 버튼
    reset_btn_ = new QPushButton("시스템 리셋");
    reset_btn_->setStyleSheet("background-color: orange; color: white; font-weight: bold; font-size: 14px;");
    reset_btn_->setFixedHeight(40);
    connect(reset_btn_, &QPushButton::clicked, this, &IntegratedMainWindow::system_reset);
    control_buttons_layout->addWidget(reset_btn_, 0, 1);
    
    // 파라미터 저장 버튼
    save_btn_ = new QPushButton("파라미터 저장");
    save_btn_->setStyleSheet("background-color: green; color: white; font-weight: bold; font-size: 14px;");
    save_btn_->setFixedHeight(40);
    connect(save_btn_, &QPushButton::clicked, this, &IntegratedMainWindow::save_parameters);
    control_buttons_layout->addWidget(save_btn_, 1, 0);
    
    // 파라미터 로드 버튼
    load_btn_ = new QPushButton("파라미터 로드");
    load_btn_->setStyleSheet("background-color: blue; color: white; font-weight: bold; font-size: 14px;");
    load_btn_->setFixedHeight(40);
    connect(load_btn_, &QPushButton::clicked, this, &IntegratedMainWindow::load_parameters);
    control_buttons_layout->addWidget(load_btn_, 1, 1);
    
    control_buttons_group->setLayout(control_buttons_layout);
    status_layout->addWidget(control_buttons_group);
    
    status_group_->setLayout(status_layout);
    main_layout_->addWidget(status_group_, 1, 1);
}

void IntegratedMainWindow::setup_control_buttons() {
    // 이 함수는 setup_status_section에서 인라인으로 구현됨
}

void IntegratedMainWindow::add_preset_buttons(QGridLayout* layout, int start_row) {
    std::unordered_map<QString, std::unordered_map<std::string, double>> presets = {
        {"보수적", {{"k_lin", 0.2}, {"k_ang", 0.05}, {"min_linear_speed", 0.3}, {"min_angular_speed", 0.3}}},
        {"표준", {{"k_lin", 0.3}, {"k_ang", 0.1}, {"min_linear_speed", 0.55}, {"min_angular_speed", 0.55}}},
        {"적극적", {{"k_lin", 0.5}, {"k_ang", 0.2}, {"min_linear_speed", 0.8}, {"min_angular_speed", 0.8}}}
    };
    
    layout->addWidget(new QLabel("프리셋:"), start_row, 0);
    
    int col = 1;
    for (const auto& [name, params] : presets) {
        QPushButton* btn = new QPushButton(name);
        btn->setStyleSheet("background-color: lightblue; font-weight: bold;");
        connect(btn, &QPushButton::clicked, [this, params]() {
            apply_preset(params);
        });
        layout->addWidget(btn, start_row, col++);
    }
}

void IntegratedMainWindow::apply_preset(const std::unordered_map<std::string, double>& preset_params) {
    for (const auto& [param_name, value] : preset_params) {
        auto it = goal_mover_spinboxes_.find(param_name);
        if (it != goal_mover_spinboxes_.end()) {
            it->second->setValue(value);
            ros_thread_->set_goal_mover_param(param_name, value);
        }
    }
}

void IntegratedMainWindow::connect_ros_signals() {
    connect(ros_thread_, &IntegratedRosThread::state_update,
            [this](const QString& state) {
                state_label_->setText(QString("전체 상태: %1").arg(state));
            });
    
    connect(ros_thread_, &IntegratedRosThread::angle_update,
            [this](double angle) {
                angle_label_->setText(QString("각도 오차: %1").arg(angle, 0, 'f', 2));
            });
    
    connect(ros_thread_, &IntegratedRosThread::dist_update,
            [this](double distance) {
                distance_label_->setText(QString("거리 오차: %1").arg(distance, 0, 'f', 2));
            });
    
    connect(ros_thread_, &IntegratedRosThread::img_update,
            this, &IntegratedMainWindow::update_camera);
    
    connect(ros_thread_, &IntegratedRosThread::robot_states_update,
            this, &IntegratedMainWindow::update_robot_states);
    
    connect(ros_thread_, &IntegratedRosThread::performance_update,
            this, &IntegratedMainWindow::update_performance);
}

void IntegratedMainWindow::update_camera(const QImage& image) {
    try {
        QPixmap pixmap = QPixmap::fromImage(image).scaled(
            camera_label_->size(),
            Qt::KeepAspectRatio,
            Qt::FastTransformation
        );
        
        camera_label_->setPixmap(pixmap);
        
        frame_count_++;
        fps_counter_++;
        
        if (frame_count_ % 30 == 0) {
            cam_status_label_->setText(QString("Status: Live - %1x%2")
                                      .arg(image.width()).arg(image.height()));
            frame_count_label_->setText(QString("Frames: %1").arg(frame_count_));
        }
        
    } catch (const std::exception& e) {
        cam_status_label_->setText(QString("Status: Error - %1").arg(e.what()));
    }
}

void IntegratedMainWindow::update_robot_states(const QVariantMap& robot_states) {
    // 맵 위젯 업데이트
    map_widget_->update_robot_positions(robot_states);
    
    // 맵 정보 레이블 업데이트
    int active_count = robot_states.size();
    map_info_label_->setText(QString("맵 크기: 2.0m x 1.0m | 활성 로봇: %1").arg(active_count));
    
    // 기존 로봇 상태 위젯들 제거
    QLayoutItem* item;
    while ((item = robot_details_layout_->takeAt(0)) != nullptr) {
        delete item->widget();
        delete item;
    }
    
    // 새로운 로봇 상태 위젯들 추가
    for (auto it = robot_states.begin(); it != robot_states.end(); ++it) {
        int robot_id = it.key().toInt();
        QVariantMap state = it.value().toMap();
        
        QGroupBox* robot_widget = new QGroupBox(QString("로봇 %1").arg(robot_id));
        QGridLayout* robot_layout = new QGridLayout();
        
        // 위치 정보
        robot_layout->addWidget(new QLabel("위치:"), 0, 0);
        robot_layout->addWidget(new QLabel(QString("(%1, %2)")
                                          .arg(state["x"].toDouble(), 0, 'f', 3)
                                          .arg(state["y"].toDouble(), 0, 'f', 3)), 0, 1);
        
        // 방향 정보
        robot_layout->addWidget(new QLabel("방향:"), 1, 0);
        robot_layout->addWidget(new QLabel(QString("%1°")
                                          .arg(state["yaw"].toDouble() * 180.0 / M_PI, 0, 'f', 1)), 1, 1);
        
        // 속도 정보
        robot_layout->addWidget(new QLabel("속도:"), 2, 0);
        double vx = state["vx"].toDouble();
        double vy = state["vy"].toDouble();
        double speed = std::sqrt(vx * vx + vy * vy);
        robot_layout->addWidget(new QLabel(QString("%1 m/s").arg(speed, 0, 'f', 3)), 2, 1);
        
        // 각속도 정보
        robot_layout->addWidget(new QLabel("각속도:"), 3, 0);
        robot_layout->addWidget(new QLabel(QString("%1°/s")
                                          .arg(state["vyaw"].toDouble() * 180.0 / M_PI, 0, 'f', 1)), 3, 1);
        
        robot_widget->setLayout(robot_layout);
        robot_details_layout_->addWidget(robot_widget);
    }
}

void IntegratedMainWindow::update_performance(const QVariantMap& performance) {
    detection_label_->setText(QString("Detections: %1")
                             .arg(performance["marker_count"].toInt()));
    
    performance_label_->setText(QString("처리 시간: %1ms")
                               .arg(performance["total_time"].toDouble(), 0, 'f', 1));
}

void IntegratedMainWindow::update_fps_display() {
    fps_label_->setText(QString("카메라 FPS: %1").arg(fps_counter_));
    fps_counter_ = 0;
}

void IntegratedMainWindow::emergency_stop() {
    // TODO: 로봇들에게 정지 명령 전송
    state_label_->setText("전체 상태: 긴급 정지됨");
    RCLCPP_WARN(node_->get_logger(), "Emergency stop activated");
}

void IntegratedMainWindow::system_reset() {
    // TODO: 시스템 리셋 로직
    state_label_->setText("전체 상태: 시스템 리셋됨");
    RCLCPP_INFO(node_->get_logger(), "System reset performed");
}

void IntegratedMainWindow::save_parameters() {
    // TODO: 현재 파라미터들을 파일로 저장
    RCLCPP_INFO(node_->get_logger(), "Parameters saved");
}

void IntegratedMainWindow::load_parameters() {
    // TODO: 파일에서 파라미터들을 로드
    RCLCPP_INFO(node_->get_logger(), "Parameters loaded");
}

void IntegratedMainWindow::closeEvent(QCloseEvent* event) {
    ros_thread_->stop_processing();
    ros_thread_->quit();
    ros_thread_->wait();
    event->accept();
}

// IntegratedArucoApp 구현
IntegratedArucoApp::IntegratedArucoApp(int argc, char** argv) {
    // ROS 초기화
    rclcpp::init(argc, argv);
    
    // Qt 애플리케이션 초기화
    qt_app_ = std::make_unique<QApplication>(argc, argv);
    
    // ROS 노드 생성
    ros_node_ = std::make_shared<rclcpp::Node>("integrated_aruco_monitor");
    
    // ROS 스레드 생성
    ros_thread_ = std::make_unique<IntegratedRosThread>(ros_node_);
    
    // 메인 윈도우 생성
    main_window_ = std::make_unique<IntegratedMainWindow>(ros_thread_.get(), ros_node_);
}

IntegratedArucoApp::~IntegratedArucoApp() {
    cleanup();
}

int IntegratedArucoApp::run() {
    try {
        // 메인 윈도우 표시
        main_window_->show();
        
        // ROS 스레드 시작
        ros_thread_->start();
        
        // Qt 이벤트 루프 실행
        int result = qt_app_->exec();
        
        return result;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Application error: %s", e.what());
        return -1;
    }
}

void IntegratedArucoApp::cleanup() {
    if (ros_thread_) {
        ros_thread_->stop_processing();
        ros_thread_->quit();
        ros_thread_->wait();
    }
    
    if (ros_node_) {
        ros_node_.reset();
    }
    
    rclcpp::shutdown();
}

// 메인 함수
int main(int argc, char** argv) {
    try {
        IntegratedArucoApp app(argc, argv);
        return app.run();
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return -1;
    }
}

// MOC 포함 (Qt 메타 오브젝트 컴파일러용)
#include "monitoring_gui.moc"