import rclpy
from rclpy.node import Node
from turtlesim.msg import Color
from std_msgs.msg import String as StringMsg
from std_srvs.srv import Empty
from turtlesim.srv import Kill, SetPen, Spawn
from turtle_tron_interfaces.srv import SetTeamColor, Respawn
import re
from typing import List, Dict


DEFAULT_SPAWN_X = 5.0
DEFAULT_SPAWN_Y = 5.0
DEFAULT_SPAWN_ORIENTATION = 0.0

class PenColor():
    def __init__(self, r, g, b, width, off) -> None:
        self.r = r
        self.g = g
        self.b = b
        self.width = width
        self.off = off

class PlayerDetails():
    
    def __init__(self, player_name: str, is_chaser: bool, team: int):
        self.name = player_name
        self.is_chaser = is_chaser
        self.team = team

class TronGame(Node):
    def __init__(self):
        super().__init__('tron_game')
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.subscribers = {}
        self.game_log_pub = self.create_publisher(StringMsg, '/turtlesim/game_log', 10)
        
        self.players: List[str] = []
        self.player_details: Dict[str, PlayerDetails] = {}
        self.eliminated_players: List[str] = []
        self.team_colors: Dict[int, PenColor] = {}
        
        self.set_team_color_srv = self.create_service(SetTeamColor, '/tron_game/set_team_color', self.set_team_color_callback)
        self.respawn_srv = self.create_service(Respawn, '/tron_game/respawn', self.respawn_callback)
        self.respawn_all_srv = self.create_service(Empty, '/tron_game/respawn_all', self.respawn_all_callback)
        
        
        self.kill_client = self.create_client(Kill, '/kill')
        self.set_pen_clients = {}  # Dictionary to store SetPen clients by player name
        self.spawn_client = self.create_client(Spawn, '/spawn')
        
        
        
        self.get_logger().info('Tron Game has been started')

    def timer_callback(self):
        topic_names_and_types = self.get_topic_names_and_types()
        pattern = re.compile(r'/(\S*)/color_sensor')
        
        self.get_logger().info('scanning for new players')
        
        for topic_name, topic_type in topic_names_and_types:
            match = pattern.match(topic_name)
            if match and topic_name not in self.subscribers:
                player_name = match.group(1)
                
                new_player_details = PlayerDetails(player_name, False, 1)
        
                self.players.append(player_name)
                self.player_details[player_name] = new_player_details
                
                self.create_dynamic_subscription(topic_name, topic_type[0], player_name)

    def create_dynamic_subscription(self, topic_name, topic_type, player_name):
        self.get_logger().info(f'Subscribing to {topic_name} of type {topic_type}')
        self.subscribers[topic_name] = self.create_subscription(
            Color,
            topic_name,
            lambda msg, pn=player_name: self.dynamic_callback(msg, pn),
            10
        )

    def dynamic_callback(self, msg, player_name):
        player_details = self.player_details.get(player_name)
        if player_details is None:
            self.get_logger().error(f'Player details not found for {player_name}')
            return
        
        players_team = player_details.team
        enemy_colors = [color for (team, color) in (self.team_colors.items()) if team !=players_team]
        for enemy_color in enemy_colors:
            if msg.r == enemy_color.r and msg.g == enemy_color.g and msg.b == enemy_color.b:
                log_msg = StringMsg()
                log_msg.data = f'Player {player_name} was eliminated'
                self.eliminated_players.append(player_name)
                self.game_log_pub.publish(log_msg)
                self.get_logger().info(log_msg.data)
                self.call_kill_service(player_name)

    def call_kill_service(self, player_name):
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /kill not available, waiting again...')
        
        request = Kill.Request()
        request.name = player_name
        
        future = self.kill_client.call_async(request)
        future.add_done_callback(lambda future: self.kill_service_callback(future, player_name))

    def set_team_color_callback(self, request, response):
        self.team_colors[request.team] = PenColor(request.r, request.g, request.b, request.width, request.off)
        for player in self.players:
            player_details = self.player_details.get(player)
            if player_details and player_details.team == request.team:
                self.call_set_pen_service(player, request.r, request.g, request.b, request.width, request.off)
        return response


    def kill_service_callback(self, future, player_name):
        try:
            response = future.result()
            self.get_logger().info(f'Successfully killed {player_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to call /kill service: {e}')

    def create_set_pen_client(self, player_name):
        if player_name not in self.set_pen_clients:
            self.set_pen_clients[player_name] = self.create_client(SetPen, f'/{player_name}/set_pen')
    
    def call_set_pen_service(self, player_name, r, g, b, width, off):
        self.create_set_pen_client(player_name)
        client = self.set_pen_clients[player_name]
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service /{player_name}/set_pen not available, waiting again...')
        
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        
        future = client.call_async(request)
        future.add_done_callback(lambda future: self.set_pen_service_callback(future, player_name))

    def set_pen_service_callback(self, future, player_name):
        try:
            response = future.result()
            self.get_logger().info(f'Successfully set pen for {player_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to call /{player_name}/set_pen service: {e}')
            
            
    def respawn_callback(self, request, response):
        player_name = request.name
        if player_name in self.eliminated_players:
            self.call_spawn_service(player_name)
            self.eliminated_players.remove(player_name)
        else:
            self.get_logger().error(f'Player {request.name} is not eliminated')
        return response     
      
    def respawn_all_callback(self, request, response):
        for name in self.eliminated_players:
            self.call_spawn_service(name)
        self.eliminated_players.clear()
            
        return response

    def call_spawn_service(self, player_name):
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /spawn not available, waiting again...')
        
        request = Spawn.Request()
        request.name = player_name
        request.x = DEFAULT_SPAWN_X
        request.y = DEFAULT_SPAWN_Y
        request.theta = DEFAULT_SPAWN_ORIENTATION
        
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda future: self.spawn_service_callback(future, player_name))

    def spawn_service_callback(self, future, player_name):
        try:
            response = future.result()
            self.get_logger().info(f'Successfully spawned {player_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to call /spawn service: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TronGame()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
