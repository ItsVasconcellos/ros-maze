#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <queue>
#include <set>
#include <tuple>
#include <memory>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/get_map.hpp"

struct Node
{
    int x, y;
    double g, h;
    std::shared_ptr<Node> parent;

    Node(int x, int y, double g, double h, std::shared_ptr<Node> parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}

    double f() const { return g + h; }

    bool operator<(const Node &other) const
    {
        return f() > other.f(); // Invertido para que a menor prioridade seja a maior f
    }
};

struct Comparator
{
    bool operator()(const std::shared_ptr<Node> &a, const std::shared_ptr<Node> &b) const
    {
        return a->f() > b->f();
    }
};

std::vector<std::pair<int, int>> get_neighbors(int x, int y)
{
    return {{x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}};
}

double heuristic(int x1, int y1, int x2, int y2)
{
    return std::abs(x1 - x2) + std::abs(y1 - y2); // Distância Manhattan
}

std::vector<std::pair<int, int>> a_star(const std::vector<std::vector<uint16_t>> &map, int start_x, int start_y, int target_x, int target_y)
{
    int height = map.size();
    int width = map[0].size();

    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, Comparator> open_list;
    std::set<std::pair<int, int>> closed_list;

    open_list.emplace(std::make_shared<Node>(start_x, start_y, 0, heuristic(start_x, start_y, target_x, target_y)));

    while (!open_list.empty())
    {
        auto current = open_list.top(); // std::shared_ptr<Node>
        open_list.pop();

        if (current->x == target_x && current->y == target_y)
        {
            // Reconstroi o caminho
            std::vector<std::pair<int, int>> path;
            for (auto n = current; n != nullptr; n = n->parent)
                path.emplace_back(n->x, n->y);
            std::reverse(path.begin(), path.end());
            return path;
        }

        closed_list.insert({current->x, current->y});

        for (auto [nx, ny] : get_neighbors(current->x, current->y))
        {
            if (nx < 0 || nx >= height || ny < 0 || ny >= width || map[nx][ny] == 0 || closed_list.count({nx, ny}))
                continue;

            double g = current->g + 1; // Assumindo custo uniforme
            double h = heuristic(nx, ny, target_x, target_y);
            open_list.emplace(std::make_shared<Node>(nx, ny, g, h, current));
        }
    }

    return {}; // Caminho não encontrado
}

class MazeClient : public rclcpp::Node
{
public:
    std::vector<std::vector<uint16_t>> map;
    std::vector<uint8_t> map_size;
    std::vector<int> start_position;
    std::vector<int> target_position;
    std::vector<std::pair<int, int>> path;
    bool map_processed_ = false;
    bool move_command_sent_ = false;

    MazeClient() : Node("maze_client")
    {
        client_move_cmd_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        client_get_map_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");

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
                                                           [this](rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future)
                                                           {
                                                               try
                                                               {
                                                                   auto response = future.get();
                                                                   if (response->success)
                                                                   {
                                                                       move_command_sent_ = true;
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
                                                          [this](rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture future)
                                                          {
                                                              try
                                                              {
                                                                  auto response = future.get();
                                                                  RCLCPP_INFO(rclcpp::get_logger("MazeClient"), "Map recieved, it has size: %d x %d", response->occupancy_grid_shape[0], response->occupancy_grid_shape[1]);
                                                                  setMap(response->occupancy_grid_shape[0], response->occupancy_grid_shape[1], response->occupancy_grid_flattened);
                                                                  map_processed_ = true;
                                                              }
                                                              catch (const std::exception &e)
                                                              {
                                                                  RCLCPP_ERROR(rclcpp::get_logger("MazeClient"), "Service call failed: %s", e.what());
                                                              }
                                                          });
    }

    void setMap(uint8_t height, uint8_t width, std::vector<std::string> occupancy_grid_flattened)
    {
        this->map_size = {height, width};
        this->map = std::vector<std::vector<uint16_t>>(height, std::vector<uint16_t>(width, 0));
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                if (occupancy_grid_flattened[i * width + j] == "b")
                {
                    this->map[i][j] = 0;
                }
                if (occupancy_grid_flattened[i * width + j] == "f")
                {
                    this->map[i][j] = 1;
                }
                if (occupancy_grid_flattened[i * width + j] == "t")
                {
                    this->target_position = {i, j};
                    this->map[i][j] = 2;
                }
                if (occupancy_grid_flattened[i * width + j] == "r")
                {
                    this->start_position = {i, j};
                    this->map[i][j] = 3;
                }
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("MazeClient"), "Map processed!");
    }

    void makePath()
    {
        if (this->map.empty())
        {
            return;
        }
        path = a_star(this->map, this->start_position[0], this->start_position[1], this->target_position[0], this->target_position[1]);
        if (!path.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Path found with %zu steps.", path.size());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No path found.");
        }
    }

    void printPath()
    {
        if (path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No path to print.");
            return;
        }

        std::stringstream ss;
        for (const auto &step : path)
        {
            ss << "(" << step.first << ", " << step.second << ") ";
        }
        RCLCPP_INFO(this->get_logger(), "Path: %s", ss.str().c_str());
    }

    void followPath()
    {
        if (path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No path to follow.");
            return;
        }
        RCLCPP_WARN(this->get_logger(), "No path to follow. %zu", path.size());

        for (size_t i = 1; i < path.size(); i++)
        {
            move_command_sent_ = false;
            int dx = path[i].first - path[i - 1].first;
            int dy = path[i].second - path[i - 1].second;

            std::string direction;
            if (dx == 1)
                direction = "down";
            else if (dx == -1)
                direction = "up";
            else if (dy == 1)
                direction = "right";
            else if (dy == -1)
                direction = "left";

            RCLCPP_INFO(this->get_logger(), "Direction %s", direction.c_str());
            sendMoveCommand(direction);
            while (!move_command_sent_)
            {
                rclcpp::spin_some(this->get_node_base_interface()); // Process callbacks
            }
        }
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_move_cmd_;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client_get_map_;
};
;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Iniciando o MazeClient...");
    auto client = std::make_shared<MazeClient>();

    client->requestMap();

    while (!client->map_processed_)
    {
        rclcpp::spin_some(client); // Process incoming callbacks and check the map_processed_ flag
    }

    client->makePath();
    client->printPath();
    client->followPath();
    rclcpp::spin(client);
    client.reset(); // Destroy the node
    rclcpp::shutdown();
}