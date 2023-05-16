
#include "transform_node.h"

namespace rosplane2
{

    transform_node::transform_node() : Node("transform_node"){

        vehicle_state_sub_ = this->create_subscription<rosplane2_msgs::msg::State>(
                "state", 10, std::bind(&transform_node::state_callback, this, _1)
                );

        static_transform_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        geometry_msgs::msg::TransformStamped static_transform; //TODO make sure this is the right message type.

        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = "enu";
        static_transform.child_frame_id = "ned";

        static_transform.transform.translation.x = 0.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;

        Eigen::Matrix3f rot;

        rot << 0.0, 1.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 0.0, -1.0;

        rot.transposeInPlace();

        Eigen::Quaternionf q(rot);

        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y(); // TODO make sure this is right.
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();

        static_transform_broadcaster->sendTransform(static_transform);

    }

    void transform_node::state_callback(const rosplane2_msgs::msg::State::SharedPtr msg) {

        auto stamp = this->get_clock()->now();
        std::vector<float> translation = {msg->position[0], msg->position[1], msg->position[2]};

        std::vector<float> zero_translation = {0.0, 0.0, 0.0};

        publish_transform(stamp, "ned", "vehicle",
                          Eigen::Matrix3f::Identity(), translation);

        Eigen::AngleAxisf rot_vehicle_to_vehicle1(msg->psi, Eigen::Vector3f(0,0,1));

        publish_transform(stamp, "vehicle", "vehicle1",
                          rot_vehicle_to_vehicle1.toRotationMatrix(), zero_translation);

        Eigen::AngleAxisf rot_vehicle1_to_vehicle2(msg->theta, Eigen::Vector3f(0,1,0));

        publish_transform(stamp, "vehicle1", "vehicle2",
                          rot_vehicle1_to_vehicle2.toRotationMatrix(), zero_translation);

        Eigen::AngleAxisf rot_vehicle2_to_body(msg->phi, Eigen::Vector3f(1,0,0));

        publish_transform(stamp, "vehicle2", "body",
                          rot_vehicle2_to_body.toRotationMatrix(),zero_translation);

        Eigen::AngleAxisf rot_body_to_stability(-msg->alpha, Eigen::Vector3f(0,1,0));

        publish_transform(stamp, "body", "stability",
                          rot_body_to_stability.toRotationMatrix(),zero_translation);

        Eigen::AngleAxisf rot_stability_to_wind(msg->beta, Eigen::Vector3f(0,0,1));

        publish_transform(stamp, "stability", "wind",
                          rot_vehicle2_to_body.toRotationMatrix(), zero_translation);

    }


    void transform_node::publish_transform(rclcpp::Time stamp, std::string header_frame, std::string child_frame,
                                           Eigen::Matrix3f rot, std::vector<float> translation) {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = header_frame;
        transform_stamped.child_frame_id = child_frame;

        transform_stamped.transform.translation.x = translation.at(0);
        transform_stamped.transform.translation.y = translation.at(1);
        transform_stamped.transform.translation.z = translation.at(2);

        Eigen::Quaternionf q(rot);

        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        transform_broadcaster->sendTransform(transform_stamped);
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rosplane2::transform_node>());

    return 0;
}
