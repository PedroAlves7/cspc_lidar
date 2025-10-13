#include "node_lidar.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <chrono>

// Função auxiliar para converter Scan para PointCloud2
void Scan_to_PointCloud(const LaserScan &scan, sensor_msgs::msg::PointCloud2 &outscan, const std::string &frame_id)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = frame_id; // Usar o frame_id passado como argumento
    cloud->width = scan.points.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < scan.points.size(); i++)
    {
        pcl::PointXYZ pcl_point;
        pcl_point.x = scan.points[i].range * std::cos(scan.points[i].angle * M_PI / 180.0);
        pcl_point.y = scan.points[i].range * std::sin(scan.points[i].angle * M_PI / 180.0);
        pcl_point.z = 0;
        cloud->points[i] = pcl_point;
    }
    pcl::toROSMsg(*cloud, outscan);
}

// Classe principal que encapsula toda a lógica do nó
class CspcLidarNode : public rclcpp::Node
{
public:
    CspcLidarNode() : Node("cspc_lidar")
    {
        // Declaração de parâmetros
        this->declare_parameter<double>("time_offset", 0.0);
        this->declare_parameter<std::string>("port", "/dev/sc_mini");
        this->declare_parameter<int>("baudrate", 230400);
        this->declare_parameter<std::string>("frame_id", "laser_frame");

        // Correção do tipo de parâmetro para 'int'
        this->declare_parameter<int>("version", 0);

        // Obtenção de parâmetros
        this->get_parameter("time_offset", time_offset_sec_);
        this->get_parameter("port", node_lidar.lidar_general_info.port);
        this->get_parameter("baudrate", node_lidar.lidar_general_info.m_SerialBaudrate);
        this->get_parameter("frame_id", node_lidar.lidar_general_info.frame_id);
        this->get_parameter("version", node_lidar.lidar_general_info.version);

        // Subscritor para comandos de status
        status_subscription_ = this->create_subscription<std_msgs::msg::UInt16>(
            "lidar_status", 10, std::bind(&CspcLidarNode::status_callback, this, std::placeholders::_1));

        // Publicadores
        error_pub_ = this->create_publisher<std_msgs::msg::String>("lsd_error", 10);
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        pcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);

        RCLCPP_INFO(this->get_logger(), "Iniciando o driver do Lidar CSPC...");

        node_start();

        lidar_thread_ = std::thread(&CspcLidarNode::lidar_loop, this);
    }

    ~CspcLidarNode()
    {
        node_lidar.serial_port->write_data(end_lidar, 4);
        if (lidar_thread_.joinable())
        {
            lidar_thread_.join();
        }
    }

private:
    void status_callback(const std_msgs::msg::UInt16::SharedPtr msg)
    {
       switch (msg->data)
       {
          case 1:
             node_lidar.lidar_status.lidar_ready = true;
             node_lidar.lidar_status.lidar_abnormal_state = 0;
             RCLCPP_INFO(this->get_logger(), "Comando recebido: Iniciar Lidar");
             break;

          case 2:
             node_lidar.lidar_status.lidar_ready = false;
             node_lidar.lidar_status.close_lidar = true;
             node_lidar.serial_port->write_data(end_lidar,4);
             RCLCPP_INFO(this->get_logger(), "Comando recebido: Parar Lidar");
             break;
        // Adicione outros casos conforme necessário
          default:
             break;
       }
    }

    void lidar_loop()
    {
        while (rclcpp::ok())
        {
            if (node_lidar.lidar_status.lidar_abnormal_state != 0)
            {
                std_msgs::msg::String pubdata;
                pubdata.data = "Erro no Lidar!";
                error_pub_->publish(pubdata);
                RCLCPP_ERROR(this->get_logger(), "Estado anormal do Lidar detectado!");
                node_lidar.serial_port->write_data(end_lidar, 4);
                node_lidar.lidar_status.lidar_ready = false;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            LaserScan scan;
            if (data_handling(scan))
            {
                auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
                sensor_msgs::msg::PointCloud2 pcl_msg;

                Scan_to_PointCloud(scan, pcl_msg, node_lidar.lidar_general_info.frame_id);

                rclcpp::Time corrected_time = this->get_clock()->now() + rclcpp::Duration::from_seconds(time_offset_sec_);

                scan_msg->header.stamp = corrected_time;
                scan_msg->header.frame_id = node_lidar.lidar_general_info.frame_id;
                scan_msg->angle_min = scan.config.min_angle;
                scan_msg->angle_max = scan.config.max_angle;
                scan_msg->angle_increment = scan.config.angle_increment;
                scan_msg->scan_time = scan.config.scan_time;
                scan_msg->time_increment = scan.config.time_increment;
                scan_msg->range_min = scan.config.min_range;
                scan_msg->range_max = scan.config.max_range;

                scan_msg->ranges.resize(scan.points.size());
                scan_msg->intensities.resize(scan.points.size());
                for (size_t i = 0; i < scan.points.size(); i++)
                {
                    scan_msg->ranges[i] = scan.points[i].range;
                    scan_msg->intensities[i] = scan.points[i].intensity;
                }

                scan_pub_->publish(std::move(scan_msg));

                pcl_msg.header.stamp = corrected_time;
                pcloud_pub_->publish(pcl_msg);
            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr status_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcloud_pub_;
    std::thread lidar_thread_;
    double time_offset_sec_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto lidar_node = std::make_shared<CspcLidarNode>();
    rclcpp::spin(lidar_node);
    rclcpp::shutdown();
    return 0;
}