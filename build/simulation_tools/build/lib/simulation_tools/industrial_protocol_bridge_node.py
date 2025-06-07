#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import yaml
import json
import threading
import time
import asyncio
import asyncua
import paho.mqtt.client as mqtt

class IndustrialProtocolBridgeNode(Node):
    def __init__(self):
        super().__init__('industrial_protocol_bridge_node')
        
        # Declare parameters
        self.declare_parameter('config_dir', '')
        self.declare_parameter('opcua_enabled', True)
        self.declare_parameter('opcua_endpoint', 'opc.tcp://0.0.0.0:4840/freeopcua/server/')
        self.declare_parameter('mqtt_enabled', True)
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('hybrid_mode', False)
        
        # Get parameters
        self.config_dir = self.get_parameter('config_dir').value
        self.opcua_enabled = self.get_parameter('opcua_enabled').value
        self.opcua_endpoint = self.get_parameter('opcua_endpoint').value
        self.mqtt_enabled = self.get_parameter('mqtt_enabled').value
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.hybrid_mode = self.get_parameter('hybrid_mode').value
        
        # Initialize state
        self.environment_config = {}
        self.system_status = 'idle'
        self.metrics = {}
        self.opcua_server = None
        self.mqtt_client = None
        
        # Create subscribers
        self.status_sub = self.create_subscription(
            String,
            '/simulation/status',
            self.status_callback,
            10)
        self.metrics_sub = self.create_subscription(
            String,
            '/simulation/metrics',
            self.metrics_callback,
            10)
        self.config_sub = self.create_subscription(
            String,
            '/simulation/config',
            self.config_callback,
            10)
        
        # Create publishers
        self.command_pub = self.create_publisher(
            String, 
            '/simulation/command', 
            10)
        
        # Start OPC UA server if enabled
        if self.opcua_enabled:
            self.opcua_thread = threading.Thread(target=self.run_opcua_server)
            self.opcua_thread.daemon = True
            self.opcua_thread.start()
        
        # Start MQTT client if enabled
        if self.mqtt_enabled:
            self.setup_mqtt_client()
        
        self.get_logger().info('Industrial protocol bridge node initialized')
        
        if self.hybrid_mode:
            self.get_logger().info('Running in hybrid mode (real camera + simulated robots)')
        
    def status_callback(self, msg):
        try:
            status_data = json.loads(msg.data)
            self.system_status = status_data.get('status', 'unknown')
            
            # Publish to MQTT if enabled
            if self.mqtt_enabled and self.mqtt_client and self.mqtt_client.is_connected():
                self.mqtt_client.publish('simulation/status', msg.data)
        except Exception as e:
            self.get_logger().error(f'Error processing status message: {e}')
    
    def metrics_callback(self, msg):
        try:
            self.metrics = json.loads(msg.data)
            
            # Publish to MQTT if enabled
            if self.mqtt_enabled and self.mqtt_client and self.mqtt_client.is_connected():
                self.mqtt_client.publish('simulation/metrics', msg.data)
        except Exception as e:
            self.get_logger().error(f'Error processing metrics message: {e}')
    
    def config_callback(self, msg):
        try:
            config_data = json.loads(msg.data)
            if 'environment' in config_data:
                self.environment_config = config_data['environment']
            
            # Publish to MQTT if enabled
            if self.mqtt_enabled and self.mqtt_client and self.mqtt_client.is_connected():
                self.mqtt_client.publish('simulation/config', msg.data)
        except Exception as e:
            self.get_logger().error(f'Error processing config message: {e}')
    
    def setup_mqtt_client(self):
        try:
            # Create MQTT client
            client_id = f'industrial_bridge_{os.getpid()}'
            self.mqtt_client = mqtt.Client(client_id=client_id)
            
            # Set up callbacks
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            # Connect to broker
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            
            # Start the loop in a separate thread
            self.mqtt_client.loop_start()
            
            self.get_logger().info(f'MQTT client connected to {self.mqtt_broker}:{self.mqtt_port}')
        except Exception as e:
            self.get_logger().error(f'Error setting up MQTT client: {e}')
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f'MQTT connected with result code {rc}')
        
        # Subscribe to command topic
        client.subscribe('simulation/command')
    
    def on_mqtt_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            
            if topic == 'simulation/command':
                # Forward command to ROS2
                cmd_msg = String()
                cmd_msg.data = payload
                self.command_pub.publish(cmd_msg)
                self.get_logger().info(f'Received MQTT command: {payload}')
        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {e}')
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warn(f'MQTT disconnected with result code {rc}')
            # Try to reconnect
            try:
                client.reconnect()
            except Exception as e:
                self.get_logger().error(f'Error reconnecting MQTT client: {e}')
    
    def run_opcua_server(self):
        # This runs in a separate thread
        asyncio.run(self.opcua_server_main())
    
    async def opcua_server_main(self):
        try:
            # Create OPC UA server
            self.opcua_server = asyncua.Server()
            await self.opcua_server.init()
            
            # Set server information
            self.opcua_server.set_endpoint(self.opcua_endpoint)
            self.opcua_server.set_server_name("Industrial Robotics Simulation OPC UA Server")
            
            # Set up namespace
            uri = "http://example.org/industrial-robotics-simulation/"
            idx = await self.opcua_server.register_namespace(uri)
            
            # Get Objects node
            objects = self.opcua_server.nodes.objects
            
            # Create simulation folder
            simulation_folder = await objects.add_folder(idx, "Simulation")
            
            # Create status variable
            self.opcua_status = await simulation_folder.add_variable(idx, "Status", "idle")
            await self.opcua_status.set_writable()
            
            # Create metrics folder
            metrics_folder = await simulation_folder.add_folder(idx, "Metrics")
            
            # Create metrics variables
            self.opcua_cycle_time = await metrics_folder.add_variable(idx, "CycleTime", 0.0)
            self.opcua_throughput = await metrics_folder.add_variable(idx, "Throughput", 0.0)
            self.opcua_accuracy = await metrics_folder.add_variable(idx, "Accuracy", 100.0)
            self.opcua_errors = await metrics_folder.add_variable(idx, "Errors", 0)
            
            # Create command method
            await simulation_folder.add_method(
                idx, "SendCommand", self.opcua_send_command,
                [asyncua.ua.VariantType.String], [asyncua.ua.VariantType.Boolean]
            )
            
            # Start server
            async with self.opcua_server:
                self.get_logger().info(f'OPC UA server started at {self.opcua_endpoint}')
                
                # Update loop
                while True:
                    # Update status
                    await self.opcua_status.write_value(self.system_status)
                    
                    # Update metrics
                    if self.metrics:
                        await self.opcua_cycle_time.write_value(self.metrics.get('cycle_time', 0.0))
                        await self.opcua_throughput.write_value(self.metrics.get('throughput', 0.0))
                        await self.opcua_accuracy.write_value(self.metrics.get('accuracy', 100.0))
                        await self.opcua_errors.write_value(self.metrics.get('errors', 0))
                    
                    # Sleep
                    await asyncio.sleep(0.5)
        
        except Exception as e:
            self.get_logger().error(f'Error in OPC UA server: {e}')
    
    async def opcua_send_command(self, parent, command):
        try:
            # Forward command to ROS2
            cmd_msg = String()
            cmd_msg.data = command
            self.command_pub.publish(cmd_msg)
            self.get_logger().info(f'Received OPC UA command: {command}')
            return True
        except Exception as e:
            self.get_logger().error(f'Error processing OPC UA command: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = IndustrialProtocolBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up MQTT client if enabled
        if node.mqtt_enabled and node.mqtt_client:
            node.mqtt_client.loop_stop()
            node.mqtt_client.disconnect()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
