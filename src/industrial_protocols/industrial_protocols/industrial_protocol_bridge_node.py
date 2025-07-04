#!/usr/bin/env python3

"""
Bridge ROS2 topics to industrial protocols.

This node expects standard `/robot/command` and `/robot/status` interfaces as described in docs/robot_integration_guide.md.
"""

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
try:
    from pymodbus.client import ModbusTcpClient
except Exception:  # pragma: no cover - optional dependency
    ModbusTcpClient = None

class IndustrialProtocolBridgeNode(Node):
    def __init__(self):
        super().__init__('industrial_protocol_bridge_node')
        
        # Declare parameters
        self.declare_parameter('config_dir', '')
        self.declare_parameter('opcua_enabled', True)
        self.declare_parameter('opcua_endpoint', 'opc.tcp://127.0.0.1:4840/freeopcua/server/')
        self.declare_parameter('mqtt_enabled', True)
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('modbus_enabled', False)
        self.declare_parameter('modbus_host', 'localhost')
        self.declare_parameter('modbus_port', 502)
        self.declare_parameter('hybrid_mode', False)
        
        # Get parameters
        self.config_dir = self.get_parameter('config_dir').value
        self.opcua_enabled = self.get_parameter('opcua_enabled').value
        self.opcua_endpoint = self.get_parameter('opcua_endpoint').value
        self.mqtt_enabled = self.get_parameter('mqtt_enabled').value
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.modbus_enabled = self.get_parameter('modbus_enabled').value
        self.modbus_host = self.get_parameter('modbus_host').value
        self.modbus_port = self.get_parameter('modbus_port').value
        self.hybrid_mode = self.get_parameter('hybrid_mode').value
        
        # Initialize state
        self.environment_config = {}
        self.system_status = 'idle'
        self.metrics = {}
        self.opcua_server = None
        self.mqtt_client = None
        self.modbus_client = None
        # Used to signal the OPC UA server thread to exit cleanly
        self._opcua_stop_event = threading.Event()
        self._modbus_stop_event = threading.Event()
        
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
            # Thread will be joined during shutdown
        
        # Start MQTT client if enabled
        if self.mqtt_enabled:
            self.setup_mqtt_client()

        # Start Modbus client if enabled
        if self.modbus_enabled:
            self.setup_modbus_client()
        
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

            # Write status to Modbus if enabled
            if self.modbus_enabled and self.modbus_client:
                regs = self._string_to_regs(self.system_status)
                self.modbus_client.write_registers(32, regs)
        except Exception as e:
            self.get_logger().error(f'Error processing status message: {e}')
    
    def metrics_callback(self, msg):
        try:
            self.metrics = json.loads(msg.data)
            
            # Publish to MQTT if enabled
            if self.mqtt_enabled and self.mqtt_client and self.mqtt_client.is_connected():
                self.mqtt_client.publish('simulation/metrics', msg.data)

            if self.modbus_enabled and self.modbus_client:
                for i, key in enumerate(['cycle_time', 'throughput', 'accuracy', 'errors']):
                    value = int(self.metrics.get(key, 0))
                    self.modbus_client.write_register(100 + i, value)
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

    def setup_modbus_client(self):
        if ModbusTcpClient is None:
            self.get_logger().error('pymodbus not installed, Modbus disabled')
            return
        try:
            self.modbus_client = ModbusTcpClient(self.modbus_host, port=self.modbus_port)
            if self.modbus_client.connect():
                self.get_logger().info(
                    f'Modbus client connected to {self.modbus_host}:{self.modbus_port}'
                )
                self.modbus_thread = threading.Thread(target=self.modbus_poll_loop)
                self.modbus_thread.daemon = True
                self.modbus_thread.start()
            else:
                self.get_logger().error(
                    f'Failed to connect to Modbus server at {self.modbus_host}:{self.modbus_port}'
                )
                self.modbus_client = None
        except Exception as e:
            self.get_logger().error(f'Error setting up Modbus client: {e}')
    
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
            self.get_logger().warning(f'MQTT disconnected with result code {rc}')
            # Try to reconnect
            try:
                client.reconnect()
            except Exception as e:
                self.get_logger().error(f'Error reconnecting MQTT client: {e}')

    def modbus_poll_loop(self):
        last_cmd = None
        while not self._modbus_stop_event.is_set():
            try:
                if not self.modbus_client:
                    break
                result = self.modbus_client.read_holding_registers(0, 16)
                if not hasattr(result, 'registers'):
                    time.sleep(0.5)
                    continue
                data = bytearray()
                for reg in result.registers:
                    data.extend(int(reg).to_bytes(2, 'big'))
                command = data.decode('utf-8').strip('\x00')
                if command and command != last_cmd:
                    msg = String()
                    msg.data = command
                    self.command_pub.publish(msg)
                    self.get_logger().info(f'Received Modbus command: {command}')
                    last_cmd = command
            except Exception as e:
                self.get_logger().error(f'Error reading Modbus command: {e}')
            time.sleep(0.5)
    
    def run_opcua_server(self):
        # This runs in a separate thread
        asyncio.run(self.opcua_server_main())

    def stop_opcua_server(self):
        """Signal the OPC UA thread to stop and wait for it."""
        self._opcua_stop_event.set()

        if hasattr(self, "opcua_thread") and self.opcua_thread.is_alive():
            self.opcua_thread.join()
        # Ensure server is stopped
        if self.opcua_server is not None:
            try:
                asyncio.run(self.opcua_server.stop())
            except Exception as e:
                self.get_logger().error(f'Error stopping OPC UA server: {e}')

    def stop_modbus_client(self):
        self._modbus_stop_event.set()
        if self.modbus_client:
            try:
                self.modbus_client.close()
            except Exception as e:
                self.get_logger().error(f'Error closing Modbus client: {e}')

    @staticmethod
    def _string_to_regs(text, length=16):
        data = text.encode('utf-8')[: length * 2]
        data += b'\x00' * (length * 2 - len(data))
        regs = []
        for i in range(0, len(data), 2):
            regs.append(int.from_bytes(data[i : i + 2], 'big'))
        return regs
    
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
                while not self._opcua_stop_event.is_set():
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

        if node.opcua_enabled:
            node.stop_opcua_server()

        if node.modbus_enabled:
            node.stop_modbus_client()

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
