import threading

import rclpy
from rclpy.node import Node

from fastapi import FastAPI
from fastapi import BackgroundTasks
import uvicorn
from pydantic import BaseModel

from custom_msg.msg import Command


class CommandRequest(BaseModel):
    id: str


class VoiceNode(Node):

    def __init__(self):
        super().__init__('voice_node')

        self.publisher_ = self.create_publisher(Command, 'command', 10)
        self.get_logger().info('VoiceNode avviato')

        self.app = FastAPI()
        self._setup_routes()

        self.server_thread = threading.Thread(
            target=self._run_server,
            daemon=True
        )
        self.server_thread.start()

        self.get_logger().info('Server FastAPI avviato su porta 8000')

    def _setup_routes(self):

        @self.app.get("/command")
        def send_command(id: str, background_tasks: BackgroundTasks):
            background_tasks.add_task(self._publish_command, id)
            return {"status": "ok", "id": id}

    def _publish_command(self, id: str):
        command = Command()
        command.command = id
        self.publisher_.publish(command)
        self.get_logger().info(f'Pubblicato Command: id={id}')

    def _run_server(self):
        uvicorn.run(
            self.app,
            host="0.0.0.0",
            port=1234,
            log_level="info"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
