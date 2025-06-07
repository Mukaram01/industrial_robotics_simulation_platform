#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import os
import signal
import threading
import sys

class WebInterfaceNode(Node):
    """
    Node for launching and managing the web interface
    """
    def __init__(self):
        super().__init__('web_interface_node')
        
        # Declare parameters using a single dictionary
        param_defaults = {
            'port': 5000,
            'host': '0.0.0.0',
        }
        self.declare_parameters('', param_defaults)
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        
        # Get path to web app
        self.web_app_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            'web_app'
        )
        
        # Start web app
        self.start_web_app()
        
        self.get_logger().info(f'Web interface started at http://{self.host}:{self.port}')
    
    def start_web_app(self):
        """
        Start Flask web app in a subprocess
        """
        try:
            # Change to web app directory
            os.chdir(self.web_app_path)
            
            # Set environment variables
            env = os.environ.copy()
            env['FLASK_APP'] = 'src/main.py'
            
            # Start Flask app
            self.web_app_process = subprocess.Popen(
                [sys.executable, '-m', 'flask', 'run', '--host', self.host, '--port', str(self.port)],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Start threads to monitor stdout and stderr
            threading.Thread(target=self.monitor_output, args=(self.web_app_process.stdout, 'stdout')).start()
            threading.Thread(target=self.monitor_output, args=(self.web_app_process.stderr, 'stderr')).start()
            
            self.get_logger().info('Web app process started')
        except Exception as e:
            self.get_logger().error(f'Failed to start web app: {str(e)}')
    
    def monitor_output(self, pipe, name):
        """
        Monitor output from web app process
        """
        for line in iter(pipe.readline, b''):
            self.get_logger().info(f'[{name}] {line.decode().strip()}')
    
    def stop_web_app(self):
        """
        Stop Flask web app
        """
        if hasattr(self, 'web_app_process'):
            self.get_logger().info('Stopping web app process')
            try:
                # Send SIGTERM to process group
                os.killpg(os.getpgid(self.web_app_process.pid), signal.SIGTERM)
                # Wait for process to terminate
                self.web_app_process.wait(timeout=5)
                self.get_logger().info('Web app process stopped')
            except Exception as e:
                self.get_logger().error(f'Failed to stop web app: {str(e)}')
                # Force kill if necessary
                try:
                    self.web_app_process.kill()
                except:
                    pass

def main(args=None):
    rclpy.init(args=args)
    
    web_interface_node = WebInterfaceNode()
    
    try:
        rclpy.spin(web_interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        web_interface_node.stop_web_app()
        web_interface_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
