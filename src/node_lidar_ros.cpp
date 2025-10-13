#include "node_lidar_ros.h" // Seus includes originais
#include "node_lidar.h"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <rclcpp/rclcpp.hpp> // Inclui rclcpp.hpp para a classe Node
#include <rclcpp/executors/multi_threaded_executor.hpp> // Inclui o Executor

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <functional>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <thread> // Para a thread de leitura do LIDAR


// Variáveis globais (mantidas para compatibilidade com o código original)
// NOTE: Em ROS 2, é melhor evitar variáveis globais como 'node_lidar'.
extern Lidar node_lidar;
extern const char end_lidar[4];
extern const char high_exposure[4];
extern const char low_exposure[4];
extern const char high_speed[4];
extern const char low_speed[4];
extern int node_start();
extern int data_handling(LaserScan &scan);
// Fim das variáveis globais

/*===================================================================*/
// CLASSE ÚNICA: LIDAR NODE
/*===================================================================*/

class LidarNode : public rclcpp::Node
{
public:
    LidarNode(const rclcpp::NodeOptions & options)
    : Node("cspc_lidar", options) // O construtor do nó
    {
        // 1. DECLARAÇÃO E OBTENÇÃO DE PARÂMETROS
        this->declare_parameter("time_offset", 0.0);
        this->get_parameter("time_offset", time_offset_sec_);

        this->declare_parameter("port", "/dev/sc_mini");
        this->get_parameter("port", node_lidar.lidar_general_info.port);

        this->declare_parameter("baudrate", 230400); // Assumindo valor padrão
        this->get_parameter("baudrate", node_lidar.lidar_general_info.m_SerialBaudrate);

        this->declare_parameter("frame_id", "laser_link");
        this->get_parameter("frame_id", node_lidar.lidar_general_info.frame_id);

        this->declare_parameter("version", ""); // Assumindo valor padrão
        this->get_parameter("version", node_lidar.lidar_general_info.version);

        // 2. CRIAÇÃO DE PUBLISHERS
        error_pub_ = this->create_publisher<std_msgs::msg::String>("lsd_error", 10);
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
        pcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);

        // 3. CRIAÇÃO DE SUBSCRIPTION (Substitui o MinimalSubscriber)
        subscription_ = this->create_subscription<std_msgs::msg::UInt16>(
            "lidar_status", 10,
            std::bind(&LidarNode::topic_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "LidarNode initialized and ready.");
    }

    // Acesso aos Publishers a partir do main loop (para manter a lógica original)
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr get_laser_pub() { return laser_pub_; }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr get_pcloud_pub() { return pcloud_pub_; }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get_error_pub() { return error_pub_; }
    double get_time_offset() const { return time_offset_sec_; }

private:
    // --- Membros ---
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcloud_pub_;
    double time_offset_sec_;

    // --- Callback do Subscriber ---
    void topic_callback(const std_msgs::msg::UInt16::SharedPtr msg)
    {
       // O código de callback original
       switch (msg->data)
       {
          case 1:
             node_lidar.lidar_status.lidar_ready = true;
             node_lidar.lidar_status.lidar_abnormal_state = 0;
             printf("#start lidar\n");
             break;
          case 2:
             node_lidar.lidar_status.lidar_ready = false;
             node_lidar.lidar_status.close_lidar = true;
             node_lidar.serial_port->write_data(end_lidar,4);
             printf("#stop lidar\n");
             break;
          case 3:
             node_lidar.serial_port->write_data(high_exposure,4);
             break;
          case 4:
             node_lidar.serial_port->write_data(low_exposure,4);
             break;
          case 5:
             node_lidar.lidar_status.lidar_abnormal_state = 0;
             break;
          case 6:
             node_lidar.serial_port->write_data(high_speed,4);
             node_lidar.lidar_general_info.frequency_max = 103;
             node_lidar.lidar_general_info.frequency_min = 97;
             break;
          case 7:
             node_lidar.serial_port->write_data(low_speed,4);
             node_lidar.lidar_general_info.frequency_max = 68;
             node_lidar.lidar_general_info.frequency_min = 52;
             break;
          default:
             break;
       }
    }
};

/* Tratamento de Ponto de Nuvem (Mantido) */
int Scan_to_PointCloud(LaserScan &scan, sensor_msgs::msg::PointCloud2 &outscan)
{
    // ... CÓDIGO ORIGINAL ...
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = "base_pcloud";
    cloud->width = scan.points.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    pcl::PointXYZ pcl_point;
    for(int i = 0;i<scan.points.size();i++)
    {
       pcl_point.x = scan.points[i].range * std::cos(scan.points[i].angle*M_PI/180.0);
       pcl_point.y = scan.points[i].range * std::sin(scan.points[i].angle*M_PI/180.0);
       pcl_point.z = 0;
       cloud->points[i] = pcl_point;
    }

    pcl::toROSMsg(*cloud, outscan);
    return 0;
}

/* Função para Executar o Nó (Substitui topic_thread) */
void spin_node(std::shared_ptr<LidarNode> node)
{
    rclcpp::spin(node);
}

/*===================================================================*/
// FUNÇÃO MAIN MODIFICADA
/*===================================================================*/

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 1. Cria a ÚNICA instância do nó
    auto node = std::make_shared<LidarNode>(rclcpp::NodeOptions());

    // 2. Inicia a thread que rodará o executor (para o Subscriber)
    // O executor deve rodar em paralelo para que o callback topic_callback seja processado
    // enquanto o loop principal (LIDAR data handling) continua rodando.
    // Usamos o executor padrão que fará o spin em uma thread.
    std::thread spin_thread(spin_node, node);
    spin_thread.detach(); // Garante que a thread rode em background

    // 3. Inicializa o Hardware LIDAR (Original)
    node_start();

    // Variáveis de publicação (obtidas do nó único)
    auto laser_pub = node->get_laser_pub();
    auto pcloud_pub = node->get_pcloud_pub();
    auto error_pub = node->get_error_pub();
    double time_offset_sec = node->get_time_offset();
    std_msgs::msg::String pubdata;

    // 4. LOOP PRINCIPAL DE LEITURA E PUBLICAÇÃO (Original)
    while(rclcpp::ok())
    {
       if(node_lidar.lidar_status.lidar_abnormal_state != 0)
       {
          // ... Lógica de ERRO original ...
          if(node_lidar.lidar_status.lidar_abnormal_state & 0x01)
          {
             pubdata.data="node_lidar is trapped\n";
             error_pub->publish(pubdata);
             printf("异常状态1---trapped\n");
          }
          if(node_lidar.lidar_status.lidar_abnormal_state & 0x02)
          {
             pubdata.data="node_lidar frequence abnormal\n";
             error_pub->publish(pubdata);
             printf("异常状态2---frequence abnormal\n");
          }
          if(node_lidar.lidar_status.lidar_abnormal_state & 0x04)
          {
             pubdata.data="node_lidar is blocked\n";
             error_pub->publish(pubdata);
             printf("异常状态3---blocked\n");
          }
          node_lidar.serial_port->write_data(end_lidar,4);
          node_lidar.lidar_status.lidar_ready = false;

          sleep(1);
       }
       LaserScan scan;

       if(data_handling(scan))
       {
          auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
          sensor_msgs::msg::PointCloud2 pclMsg;
          Scan_to_PointCloud(scan,pclMsg);

          rclcpp::Time current_time = node->get_clock()->now();

          rclcpp::Duration offset_duration{std::chrono::duration<double>(time_offset_sec)};
          rclcpp::Time corrected_time = current_time + offset_duration;

          scan_msg->header.stamp = corrected_time;

          // ... Resto da formatação da LaserScan ...
          scan_msg->ranges.resize(scan.points.size());
          scan_msg->intensities.resize(scan.points.size());

          scan_msg->header.frame_id = node_lidar.lidar_general_info.frame_id;

          scan_msg->angle_min = scan.config.min_angle;
          scan_msg->angle_max = scan.config.max_angle;
          scan_msg->angle_increment = scan.config.angle_increment;
          scan_msg->scan_time = scan.config.scan_time;
          scan_msg->time_increment = scan.config.time_increment;
          scan_msg->range_min = scan.config.min_range;
          scan_msg->range_max = scan.config.max_range;
          for(int i=0; i < scan.points.size(); i++) {
             scan_msg->ranges[i] = scan.points[i].range;
             scan_msg->intensities[i] = scan.points[i].intensity;
          }

          laser_pub->publish(*scan_msg);
          pclMsg.header.stamp = corrected_time;
          pcloud_pub->publish(pclMsg);
        }
    }

    // 5. Shutdown (limpeza)
    node_lidar.serial_port->write_data(end_lidar,4);
    rclcpp::shutdown();

    // Não precisamos de spin_thread.join() porque usamos detach() e rclcpp::shutdown()
    return 0;
}