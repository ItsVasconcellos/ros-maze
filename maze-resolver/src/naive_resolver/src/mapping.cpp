#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/get_map.hpp"

class MazeClient : public rclcpp::Node
{
public:
    
    MazeClient() : Node("maze_client")
    {

        client_move_cmd_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        client_get_map_ = this->create_client<cg_interfaces::srv::GetMap>("get_map");

        RCLCPP_INFO(this->get_logger(), "MazeClient iniciado!");

        while (!client_move_cmd_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Aguardando a disponibilidade do serviço move_command... ");
        }
        while (!client_get_map_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Aguardando a disponibilidade do serviço get_map...");
        }
    }

    void sendMoveCommand(const std::string &direction)
    {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;

        auto result = client_move_cmd_->async_send_request(request,
                                                           [](rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future)
                                                           {
                                                               try
                                                               {
                                                                   auto response = future.get();
                                                                   if (response->success)
                                                                   {
                                                                       RCLCPP_INFO(rclcpp::get_logger("MazeClient"), "Movimento realizado com sucesso!");
                                                                   }
                                                                   else
                                                                   {
                                                                       RCLCPP_WARN(rclcpp::get_logger("MazeClient"), "Falha no movimento.");
                                                                   }
                                                               }
                                                               catch (const std::exception &e)
                                                               {
                                                                   RCLCPP_ERROR(rclcpp::get_logger("MazeClient"), "Service call failed: %s", e.what());
                                                               }
                                                           });
    }

    void requestMap()
    {
        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();

        auto result = client_get_map_->async_send_request(request,
                                                          [](rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture future)
                                                          {
                                                              try
                                                              {
                                                                  auto response = future.get();
                                                                  RCLCPP_INFO(rclcpp::get_logger("MazeClient"), "Mapa recebido: ");
                                                                  // Aqui você pode processar o mapa recebido
                                                              }
                                                              catch (const std::exception &e)
                                                              {
                                                                  RCLCPP_ERROR(rclcpp::get_logger("MazeClient"), "Service call failed: %s", e.what());
                                                              }
                                                          });
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_move_cmd_;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client_get_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Iniciando o MazeClient...");
    auto client = std::make_shared<MazeClient>();

    client->requestMap();

    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}