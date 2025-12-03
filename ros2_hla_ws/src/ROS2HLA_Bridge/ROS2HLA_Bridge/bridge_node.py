import rclpy
from rclpy.node import Node
import yaml
import importlib
import os
import time
import jpype
from ROS2HLA_Bridge.hla_manager import HLAManager
from ROS2HLA_Bridge.fom_generator import FOMGenerator
import tempfile

def get_nested_attr(obj, attr_path):
    attributes = attr_path.split('.')
    for attr in attributes:
        obj = getattr(obj, attr)
    return obj

def set_nested_attr(obj, attr_path, value):
    attributes = attr_path.split('.')
    for attr in attributes[:-1]:
        obj = getattr(obj, attr)
    # Basic type conversion if needed, but usually Python handles it or we need explicit casting
    # For now assume types match or are compatible (float->float)
    setattr(obj, attributes[-1], value)

def get_ros_class(ros_type_str):
    # e.g. "geometry_msgs.msg.Twist"
    parts = ros_type_str.split('.')
    module_name = '.'.join(parts[:-1])
    class_name = parts[-1]
    module = importlib.import_module(module_name)
    return getattr(module, class_name)

class ROS2HLABridge(Node):
    def __init__(self):
        super().__init__('ros2_hla_bridge')
        
        # Load Config
        self.declare_parameter('config_file', '')
        self.declare_parameter('robot_name', 'Turtle1') # Default
        
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.action_cb_group = ReentrantCallbackGroup()
        
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        if not config_file:
            self.get_logger().error("No config_file parameter provided!")
            return 

        with open(config_file, 'r') as f:
            raw_config = yaml.safe_load(f)
            
        # Apply templating
        self.config = self.apply_template(raw_config, {'robot_name': robot_name})
            
        # Generate FOM
        fom_gen = FOMGenerator()
        # Create a temp file for FOM
        # Unique per run is safer for concurrent tests, but HLA needs consistent FOM.
        # Actually, if we generate it, it should be consistent if config is consistent.
        fd, fom_path = tempfile.mkstemp(suffix='.xml', prefix='generated_fom_')
        os.close(fd)
        fom_gen.generate(self.config, fom_path)
        self.config['hla']['fom_file_path'] = fom_path
        self.get_logger().info(f"Generated FOM at {fom_path}")

        # Initialize HLA Manager
        self.hla_manager = HLAManager(self.config, self.get_logger())
        self.hla_manager.connect()
        self.hla_manager.create_and_join_federation()
        
        # Enable Time Management
        self.hla_manager.enable_time_management()
        
        # Sync Points
        sync_points = self.config.get('sync_points', [])
        if sync_points:
            for sp in sync_points:
                if self.hla_manager.is_regulating:
                    self.hla_manager.register_sync_point(sp)
                self.hla_manager.wait_for_sync_point(sp)
        
        # Setup ROS2 -> HLA
        self.setup_ros_to_hla()
        
        # Setup HLA -> ROS2
        self.setup_hla_to_ros()
        
        import threading
        self.hla_lock = threading.Lock()

        # Setup Actions
        self.setup_actions()
        
        # Timer for HLA callbacks
        # If time management is enabled, this loop will drive the time advance
        self.create_timer(0.01, self.hla_loop)
        
        self.get_logger().info(f"ROS2-HLA Bridge Initialized for {robot_name}")

    def apply_template(self, data, context):
        if isinstance(data, dict):
            return {k: self.apply_template(v, context) for k, v in data.items()}
        elif isinstance(data, list):
            return [self.apply_template(i, context) for i in data]
        elif isinstance(data, str):
            for k, v in context.items():
                data = data.replace(f"{{{{{k}}}}}", v)
            return data
        return data

    def setup_ros_to_hla(self):
        ros_to_hla = self.config.get('ros_to_hla', [])
        self.hla_manager.setup_publications(ros_to_hla)
        
        self.subs = []
        for item in ros_to_hla:
            topic = item['ros_topic']
            msg_type = get_ros_class(item['ros_type'])
            
            def callback(msg, item=item):
                self.ros_to_hla_callback(msg, item)
            
            sub = self.create_subscription(msg_type, topic, callback, 10, callback_group=self.action_cb_group)
            self.subs.append(sub)
            self.get_logger().info(f"Bridging {topic} -> HLA {item.get('hla_object_class') or item.get('hla_interaction_class')}")

    def setup_hla_to_ros(self):
        self.get_logger().info("DEBUG: Starting setup_hla_to_ros")
        hla_to_ros_config = self.config.get('hla_to_ros', [])
        
        self.hla_object_map = {} 
        self.pubs = {} # topic -> publisher
        
        # We need to apply templates to the config items before passing to setup_subscriptions
        processed_config = []

        for item in hla_to_ros_config:
            self.get_logger().info(f"DEBUG: Processing item {item}")
            
            # Apply template to ros_topic
            # The config is already templated in __init__, so direct access is fine.
            topic = item['ros_topic']
            
            # Create publisher
            msg_type = get_ros_class(item['ros_type'])
            pub = self.create_publisher(msg_type, topic, 10)
            self.pubs[topic] = pub
            self.get_logger().info(f"Bridging HLA {item.get('hla_interaction_class', 'Object')} -> {topic}")
            
            # Prepare processed item for subscription
            processed_item = item.copy()
            processed_item['ros_topic'] = topic
            # hla_instance_name is also already templated if it was in the config.
            
            processed_config.append(processed_item)

            # Store mapping for callback
            if 'hla_object_class' in item:
                self.hla_object_map[item['hla_object_class']] = {
                    'ros_topic': topic,
                    'msg_type': msg_type,
                    'mapping': item['mapping'],
                    'instance_name': processed_item.get('hla_instance_name', '')
                }

        # Register callbacks with HLA Manager
        self.hla_manager.on_interaction_received = self.hla_interaction_callback
        self.hla_manager.on_object_update_received = self.hla_object_update_callback
        
        self.get_logger().info("DEBUG: Calling setup_subscriptions")
        self.hla_manager.setup_subscriptions(processed_config)
        self.get_logger().info("DEBUG: Finished setup_hla_to_ros")

    def ros_to_hla_callback(self, msg, config_item):
        if not jpype.isThreadAttachedToJVM():
            jpype.attachThreadToJVM()

        # Extract data from ROS msg based on mapping
        # self.get_logger().info(f"Received ROS msg on {config_item['ros_topic']}")
        
        data_map = {}
        for ros_field, hla_param in config_item['mapping'].items():
            value = get_nested_attr(msg, ros_field) # Assuming get_ros_field_value is get_nested_attr
            if value is not None:
                data_map[hla_param] = value

        # Add fixed parameters if any
        if 'fixed_parameters' in config_item:
            for param, value in config_item['fixed_parameters'].items():
                data_map[param] = value

        # Send to HLA
        if 'hla_object_class' in config_item:
            self.hla_manager.update_object_attributes(
                config_item['hla_instance_name'],
                config_item['hla_object_class'],
                data_map
            )
        elif 'hla_interaction_class' in config_item:
            self.hla_manager.send_interaction(
                config_item['hla_interaction_class'],
                data_map
            )

    def hla_object_update_callback(self, objectInstance, attributeValues):
        obj_name = self.hla_manager.get_object_name(objectInstance)
        if not obj_name:
            return

        # Find matching config
        for item in self.config['hla_to_ros']:
            if 'hla_object_class' in item and item['hla_instance_name'] == obj_name:
                # Decode and publish
                msg = get_ros_class(item['ros_type'])() 
                
                for ros_field, hla_attr in item['mapping'].items():
                    if (item['hla_object_class'], hla_attr) in self.hla_manager.attribute_handles:
                        handle = self.hla_manager.attribute_handles[(item['hla_object_class'], hla_attr)]
                        if attributeValues.containsKey(handle):
                            val_bytes = attributeValues.get(handle)
                            val = self.hla_manager.decode_value(val_bytes, 'float')
                            set_nested_attr(msg, ros_field, val)
                
                publisher = self.pubs[item['ros_topic']]
                publisher.publish(msg)

    def hla_interaction_callback(self, interactionClass, parameterValues):
        interaction_name = None
        for name, handle in self.hla_manager.interaction_class_handles.items():
            if handle.equals(interactionClass):
                interaction_name = name
                break
        
        if not interaction_name:
            return

        # Check if it matches any hla_to_ros mapping
        for item in self.config['hla_to_ros']:
            if 'hla_interaction_class' in item and item['hla_interaction_class'] == interaction_name:
                # Check filter
                if 'filter_parameter' in item:
                    p_handle = self.hla_manager.parameter_handles[(interaction_name, item['filter_parameter'])]
                    if parameterValues.containsKey(p_handle):
                        val_bytes = parameterValues.get(p_handle)
                        val = self.hla_manager.decode_value(val_bytes, 'string')
                        if val != item['filter_value']:
                            continue
                
                # Decode and publish
                msg = get_ros_class(item['ros_type'])() 
                
                # Set timestamp if header exists
                if hasattr(msg, 'header'):
                    msg.header.stamp = self.get_clock().now().to_msg()
                    if not msg.header.frame_id:
                         msg.header.frame_id = 'base_link'
                
                # Map parameters
                for ros_field, hla_param in item['mapping'].items():
                    if (interaction_name, hla_param) in self.hla_manager.parameter_handles:
                        handle = self.hla_manager.parameter_handles[(interaction_name, hla_param)]
                        if parameterValues.containsKey(handle):
                            val_bytes = parameterValues.get(handle)
                            val = self.hla_manager.decode_value(val_bytes, 'float')
                            set_nested_attr(msg, ros_field, val)
                
                publisher = self.pubs[item['ros_topic']]
                publisher.publish(msg)

    def setup_actions(self):
        self.get_logger().info("DEBUG: Starting setup_actions")
        actions_config = self.config.get('ros_actions', [])
        
        self.action_map = {} # goal_interaction -> config_item
        self.action_clients = {} # action_name -> ActionClient
        self.pending_action_goals = {} # goal_id -> Event
        
        # Callback group for actions to allow reentrancy
        # self.action_cb_group = ReentrantCallbackGroup() # Initialized in __init__
        
        for item in actions_config:
            # 1. Subscribe to Goal Interaction (Server side)
            goal_int = item['hla_goal_interaction']
            self.action_map[goal_int] = item
            
            if goal_int not in self.hla_manager.interaction_class_handles:
                 try:
                    handle = self.hla_manager.rti_ambassador.getInteractionClassHandle(goal_int)
                    self.hla_manager.interaction_class_handles[goal_int] = handle
                    self.hla_manager.rti_ambassador.subscribeInteractionClass(handle)
                    self.get_logger().info(f"Subscribed to Action Goal: {goal_int}")
                    
                    for _, param in item['mapping'].get('goal', {}).items():
                        p_handle = self.hla_manager.rti_ambassador.getParameterHandle(handle, param)
                        self.hla_manager.parameter_handles[(goal_int, param)] = p_handle
                 except Exception as e:
                    self.get_logger().error(f"Failed to subscribe to action goal {goal_int}: {e}")

            # Subscribe to Result Interaction
            res_int = item['hla_result_interaction']
            self.action_map[res_int] = item
            
            if res_int not in self.hla_manager.interaction_class_handles:
                 try:
                    handle = self.hla_manager.rti_ambassador.getInteractionClassHandle(res_int)
                    self.hla_manager.interaction_class_handles[res_int] = handle
                    self.hla_manager.rti_ambassador.subscribeInteractionClass(handle)
                    self.get_logger().info(f"Subscribed to Action Result: {res_int}")
                    
                    for _, param in item['mapping'].get('result', {}).items():
                        p_handle = self.hla_manager.rti_ambassador.getParameterHandle(handle, param)
                        self.hla_manager.parameter_handles[(res_int, param)] = p_handle
                 except Exception as e:
                    self.get_logger().error(f"Failed to subscribe to action result {res_int}: {e}")

            # 2. Create ROS Action Client (Server side) or Server (Client side)
            action_type = get_ros_class(item['ros_type'])
            action_name = item['ros_action']
            
            if self.hla_manager.is_constrained:
                # Consumer: Expose Action Server to ROS (Client Bridge)
                # When ROS sends goal -> Send HLA Goal Interaction
                from rclpy.action import ActionServer
                self.action_server = ActionServer(
                    self,
                    action_type,
                    action_name,
                    lambda goal_handle, it=item: self.ros_action_execute_callback(goal_handle, it),
                    callback_group=self.action_cb_group
                )
                self.get_logger().info(f"Created Action Server (Consumer): {action_name}")
                
                # Publish Goal Interaction
                handle = self.hla_manager.rti_ambassador.getInteractionClassHandle(goal_int)
                self.hla_manager.interaction_class_handles[goal_int] = handle
                self.hla_manager.rti_ambassador.publishInteractionClass(handle)
                
            elif self.hla_manager.is_regulating:
                # Provider: Call real ROS Action (Server Bridge)
                from rclpy.action import ActionClient
                client = ActionClient(self, action_type, action_name, callback_group=self.action_cb_group)
                self.action_clients[action_name] = client
                self.get_logger().info(f"Prepared Action Client (Provider): {action_name}")
                
                # Publish Result Interaction
                handle = self.hla_manager.rti_ambassador.getInteractionClassHandle(res_int)
                self.hla_manager.interaction_class_handles[res_int] = handle
                self.hla_manager.rti_ambassador.publishInteractionClass(handle)

    def ros_action_execute_callback(self, goal_handle, item):
        self.get_logger().info(f"Received ROS Action Goal for {item['ros_action']}")
        
        # 1. Map Goal to HLA Interaction
        data_map = {}
        for ros_field, hla_param in item['mapping'].get('goal', {}).items():
            val = get_nested_attr(goal_handle.request, ros_field)
            data_map[hla_param] = val

        # Add fixed parameters if any
        if 'fixed_parameters' in item:
            for param, value in item['fixed_parameters'].items():
                data_map[param] = value
            
        # 2. Send Goal Interaction
        goal_int = item['hla_goal_interaction']
        self.get_logger().info(f"Sending HLA Interaction {goal_int} with data {data_map}")
        
        if not jpype.isThreadAttachedToJVM():
            jpype.attachThreadToJVM()
            
        with self.hla_lock:
            self.hla_manager.send_interaction(goal_int, data_map)
        
        # 3. Wait for Result
        req_id = "action_req" # Simplification
        
        # Use a threading Event to wait without busy looping on spin_once
        # The spin_once is handled by the main timer loop in another thread
        import threading
        event = threading.Event()
        self.pending_action_goals[req_id] = {'event': event, 'data': None}
        
        # Wait indefinitely (or until node shutdown)
        while not event.is_set():
            if not rclpy.ok():
                goal_handle.abort()
                action_type = get_ros_class(item['ros_type'])
                return action_type.Result()
            event.wait(1.0) # Check rclpy.ok() every second
            
        # Fill result
        hla_res_data = self.pending_action_goals.pop(req_id)['data']
        action_type = get_ros_class(item['ros_type'])
        result = action_type.Result()
        
        for ros_field, hla_param in item['mapping'].get('result', {}).items():
            if hla_param in hla_res_data:
                val = hla_res_data[hla_param]
                # Type conversion: HLA float -> bool?
                if isinstance(val, float) and (ros_field == 'is_docked' or ros_field == 'sees_dock'):
                     val = bool(val)
                set_nested_attr(result, ros_field, val)
        
        goal_handle.succeed()
        return result

    def hla_interaction_callback(self, interactionClass, parameterValues):
        # Check actions
        for int_name, item in self.action_map.items():
            target_handle = self.hla_manager.interaction_class_handles.get(int_name)
            if target_handle and interactionClass.equals(target_handle):
                
                if int_name == item['hla_goal_interaction']:
                    self.handle_action_goal_interaction(item, parameterValues)
                    return
                
                if int_name == item['hla_result_interaction']:
                    self.handle_action_result_interaction(item, parameterValues)
                    return

        # Existing logic...
        for item in self.config.get('hla_to_ros', []):
            if 'hla_interaction_class' not in item: continue
            
            int_name = item['hla_interaction_class']
            target_handle = self.hla_manager.interaction_class_handles.get(int_name)
            
            if target_handle and interactionClass.equals(target_handle):
                # Match found!
                
                # Check filter
                if 'filter_parameter' in item:
                    filter_param = item['filter_parameter']
                    filter_val = item['filter_value']
                    
                    p_handle = self.hla_manager.parameter_handles.get((int_name, filter_param))
                    if p_handle and parameterValues.containsKey(p_handle):
                        val_bytes = parameterValues.get(p_handle)
                        decoded_val = self.hla_manager.decode_value(val_bytes, 'string')
                        if decoded_val != filter_val:
                            continue 
                    else:
                        continue 
                
                # Create ROS message
                msg_type = get_ros_class(item['ros_type'])
                msg = msg_type()
                
                # Set timestamp if header exists
                if hasattr(msg, 'header'):
                    msg.header.stamp = self.get_clock().now().to_msg()
                    if not msg.header.frame_id:
                         msg.header.frame_id = 'base_link'
                
                # Map parameters
                for ros_field, hla_param in item['mapping'].items():
                    p_handle = self.hla_manager.parameter_handles.get((int_name, hla_param))
                    if p_handle and parameterValues.containsKey(p_handle):
                        val_bytes = parameterValues.get(p_handle)
                        # Infer type. For now assume float for control
                        decoded_val = self.hla_manager.decode_value(val_bytes, 'float')
                        set_nested_attr(msg, ros_field, decoded_val)
                
                # Publish
                if item['ros_topic'] in self.pubs:
                    self.pubs[item['ros_topic']].publish(msg)

    def handle_action_goal_interaction(self, item, parameterValues):
        self.get_logger().info(f"Received HLA Action Goal for {item['ros_action']}")
        
        action_type = get_ros_class(item['ros_type'])
        goal_msg = action_type.Goal()
        
        for ros_field, hla_param in item['mapping'].get('goal', {}).items():
            p_handle = self.hla_manager.parameter_handles.get((item['hla_goal_interaction'], hla_param))
            if p_handle and parameterValues.containsKey(p_handle):
                val_bytes = parameterValues.get(p_handle)
                val = self.hla_manager.decode_value(val_bytes, 'float')
                set_nested_attr(goal_msg, ros_field, val)

        action_name = item['ros_action']
        if action_name not in self.action_clients:
             self.get_logger().error(f"No action client for {action_name}")
             return
             
        client = self.action_clients[action_name]

        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f"Action server {action_name} not available")
            return

        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f, it=item: self.on_action_goal_accepted(f, it))

    def on_action_goal_accepted(self, future, item):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            # Send rejection result to HLA
            action_name = item['ros_action']
            if action_name in self.action_map:
                config = self.action_map[action_name]
                result_interaction = config['hla_result_interaction']
                
                hla_data = {}
                if 'Undock' in action_name:
                     hla_data['isDocked'] = True 
                
                # Send it
                if not jpype.isThreadAttachedToJVM():
                    jpype.attachThreadToJVM()
                self.hla_manager.send_interaction(result_interaction, hla_data)
            return

        self.get_logger().info('Goal accepted')
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(lambda f, it=item: self.on_action_result(f, it))

    def on_action_result(self, future, item):
        result = future.result().result
        self.get_logger().info("Got ROS Action Result, sending back to HLA")
        
        data_map = {}
        for ros_field, hla_param in item['mapping'].get('result', {}).items():
            val = get_nested_attr(result, ros_field)
            # Convert bool to float for HLA
            if isinstance(val, bool):
                val = 1.0 if val else 0.0
            data_map[hla_param] = val
            
        if not jpype.isThreadAttachedToJVM():
            jpype.attachThreadToJVM()

        with self.hla_lock:
            self.hla_manager.send_interaction(item['hla_result_interaction'], data_map)

    def handle_action_result_interaction(self, item, parameterValues):
        self.get_logger().info("Received HLA Action Result")
        
        data = {}
        for _, hla_param in item['mapping'].get('result', {}).items():
             p_handle = self.hla_manager.parameter_handles.get((item['hla_result_interaction'], hla_param))
             if p_handle and parameterValues.containsKey(p_handle):
                val_bytes = parameterValues.get(p_handle)
                val = self.hla_manager.decode_value(val_bytes, 'float')
                data[hla_param] = val
        
        req_id = "action_req"
        if req_id in self.pending_action_goals:
            self.pending_action_goals[req_id]['data'] = data
            self.pending_action_goals[req_id]['event'].set()

    def hla_loop(self):
        if not jpype.isThreadAttachedToJVM():
            jpype.attachThreadToJVM()
            
        with self.hla_lock:
            try:
                if self.hla_manager.is_regulating or self.hla_manager.is_constrained:
                    self.hla_manager.advance_time()
                else:
                    self.hla_manager.spin_once()
            except Exception as e:
                self.get_logger().error(f"HLA spin error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ROS2HLABridge()
    
    # Use MultiThreadedExecutor to allow actions to block while other callbacks run
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
