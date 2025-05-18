#include "depthai_filters/spatial_bb.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "depthai_filters/utils.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "opencv2/opencv.hpp"

namespace depthai_filters {

SpatialBB::SpatialBB(const rclcpp::NodeOptions& options) : rclcpp::Node("spatial_bb_node", options) {
    onInit();
}
void SpatialBB::onInit() {
    previewSub.subscribe(this, "rgb/preview/image_raw");
    infoSub.subscribe(this, "stereo/camera_info");
    detSub.subscribe(this, "nn/spatial_detections");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, infoSub, detSub);
    sync->registerCallback(std::bind(&SpatialBB::overlayCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    markerPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("spatial_bb", 10);
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);
    desqueeze = this->declare_parameter<bool>("desqueeze", false);
    labelMap = this->declare_parameter<std::vector<std::string>>("label_map", labelMap);
}

void SpatialBB::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
                          const vision_msgs::msg::Detection3DArray::ConstSharedPtr& detections) {
    cv::Mat previewMat = utils::msgToMat(this->get_logger(), preview, sensor_msgs::image_encodings::BGR8);

    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    float xText = 5.0;
    float yText = 10.0;
    float yTextStep = 14.0;
    float yTextShift = 0.0;

    for(auto& detection : detections->detections) {

        if(detection.results.size() == 0) {
            continue;
        }

        auto confidence = detection.results[0].hypothesis.score;
        if(confidence < 0.5) {
            continue;
        }

        // Overlay image publishing (topic "/overlay") for 2D visualization:
        auto labelStr = labelMap[stoi(detection.results[0].hypothesis.class_id)];
        std::stringstream confStr;
        confStr << labelStr << " : " << std::fixed << std::setprecision(0) << confidence * 100 << "%";
        utils::addTextToFrame(previewMat, confStr.str(), xText, yText + yTextShift);
        yTextShift += yTextStep;

        std::stringstream depthX;
        depthX << "X: " << std::fixed << std::setprecision(2) << detection.results[0].pose.pose.position.x << " m";
        utils::addTextToFrame(previewMat, depthX.str(), xText, yText + yTextShift);
        yTextShift += yTextStep;

        std::stringstream depthY;
        depthY << "Y: " << std::fixed << std::setprecision(2) << detection.results[0].pose.pose.position.y << " m";
        utils::addTextToFrame(previewMat, depthY.str(), xText, yText + yTextShift);
        yTextShift += yTextStep;

        std::stringstream depthZ;
        depthZ << "Z: " << std::fixed << std::setprecision(2) << detection.results[0].pose.pose.position.z << " m";
        utils::addTextToFrame(previewMat, depthZ.str(), xText, yText + yTextShift);
        yTextShift += yTextStep + 3.0;  // Add some space after the last text

        // Marker array publishing (topic "/spatial_bb") for limited 3D visualization:
        const auto& bbox = detection.bbox;
        auto bbox_size_x = bbox.size.x;
        auto bbox_size_y = bbox.size.y;
        //auto bbox_size_z = bbox.size.z;

        auto bbox_center_x = bbox.center.position.x;
        auto bbox_center_y = bbox.center.position.y;
        auto bbox_center_z = bbox.center.position.z;

        visualization_msgs::msg::Marker box_marker;
        box_marker.header.frame_id = info->header.frame_id;
        box_marker.header.stamp = this->get_clock()->now();
        box_marker.ns = "detections";
        box_marker.id = id++;
        box_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        box_marker.action = visualization_msgs::msg::Marker::ADD;

        box_marker.scale.x = 0.02;  // Marker line width
        box_marker.color.g = 1.0;
        box_marker.color.a = 1.0;

        // Define bbox corner points in depth image frame
        geometry_msgs::msg::Point32 corners[4];
        corners[0].x = bbox_center_x - bbox_size_x / 2.0;
        corners[0].y = bbox_center_y - bbox_size_y / 2.0;
        corners[0].z = bbox_center_z;
        corners[1].x = bbox_center_x + bbox_size_x / 2.0;
        corners[1].y = bbox_center_y - bbox_size_y / 2.0;
        corners[1].z = bbox_center_z;
        corners[2].x = bbox_center_x + bbox_size_x / 2.0;
        corners[2].y = bbox_center_y + bbox_size_y / 2.0;
        corners[2].z = bbox_center_z;
        corners[3].x = bbox_center_x - bbox_size_x / 2.0;
        corners[3].y = bbox_center_y + bbox_size_y / 2.0;
        corners[3].z = bbox_center_z;

        // The polygon points are a rectangle, so we need 5 points to close the loop
        box_marker.points.resize(5);
        for(int i = 0; i < 4; ++i) {
            auto& point = corners[i];
            box_marker.points[i].x = point.x;
            box_marker.points[i].y = point.y;
            box_marker.points[i].z = point.z;
        }
        // Repeat the first point to close the loop
        box_marker.points[4] = box_marker.points[0];
        marker_array.markers.push_back(box_marker);

        // Create a text marker for the label
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = info->header.frame_id;
        text_marker.header.stamp = this->get_clock()->now();
        text_marker.ns = "detections_label";
        text_marker.id = id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        text_marker.scale.z = 0.1;  // Text size
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        // Position the text above the bounding box
        text_marker.pose.position.x = box_marker.points[0].x;
        text_marker.pose.position.y = box_marker.points[0].y;
        text_marker.pose.position.z = box_marker.points[0].z + 0.1;  // Adjust this value to position the text above the box

        // Set the text to the detection label
        std::stringstream markerStr;
        markerStr << labelStr << ":" << std::fixed << std::setprecision(0) << confidence * 100 << "%";
        text_marker.text = markerStr.str();

        marker_array.markers.push_back(text_marker);
    }

    int current_marker_count = id;

    // Delete old markers if needed, they are not deleted automatically:
    for(int old_id = current_marker_count; old_id < last_marker_count_; ++old_id) {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = info->header.frame_id;
        delete_marker.header.stamp = this->get_clock()->now();
        delete_marker.ns = "detections";
        delete_marker.id = old_id;
        delete_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(delete_marker);

        // Also delete label markers
        visualization_msgs::msg::Marker delete_label_marker;
        delete_label_marker.header.frame_id = info->header.frame_id;
        delete_label_marker.header.stamp = this->get_clock()->now();
        delete_label_marker.ns = "detections_label";
        delete_label_marker.id = old_id;
        delete_label_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(delete_label_marker);
    }
    last_marker_count_ = current_marker_count;

    // Publish the marker array (topic "/spatial_bb") for limited 3D visualization:
    markerPub->publish(marker_array);

    // Overlay image publishing (topic "/overlay") for 2D visualization:
    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, previewMat).toImageMsg(outMsg);
    overlayPub->publish(outMsg);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::SpatialBB);
