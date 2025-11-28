import rclpy
from rclpy.node import Node
import yaml
import importlib
import os
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
        # We use a fixed name in /tmp to avoid clutter or unique per run?
        # Unique per run is safer for concurrent tests, but HLA needs consistent FOM.
        # Actually, if we generate it, it should be consistent if config is consistent.
        # Let's use a temp file.
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
        
        # Setup ROS2 -> HLA
        self.setup_ros_to_hla()
        
        # Setup HLA -> ROS2
        self.setup_hla_to_ros()
        
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
            
            sub = self.create_subscription(msg_type, topic, callback, 10)
            self.subs.append(sub)
            self.get_logger().info(f"Bridging {topic} -> HLA {item.get('hla_object_class') or item.get('hla_interaction_class')}")

    def setup_hla_to_ros(self):
        hla_to_ros = self.config.get('hla_to_ros', [])
        self.hla_manager.setup_subscriptions(hla_to_ros)
        
        self.pubs = {} # topic -> publisher
        for item in hla_to_ros:
            topic = item['ros_topic']
            msg_type = get_ros_class(item['ros_type'])
            pub = self.create_publisher(msg_type, topic, 10)
            self.pubs[topic] = pub
            self.get_logger().info(f"Bridging HLA {item.get('hla_interaction_class', 'Object')} -> {topic}")

        # Register callbacks with HLA Manager
        self.hla_manager.on_interaction_received = self.hla_interaction_callback
        self.hla_manager.on_object_update_received = self.hla_object_update_callback

    def ros_to_hla_callback(self, msg, config_item):
        # Extract data from ROS msg based on mapping
        data_map = {}
        for ros_field, hla_attr in config_item['mapping'].items():
            val = get_nested_attr(msg, ros_field)
            data_map[hla_attr] = val
            
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
            # We haven't discovered it yet or it's unknown
            return

        for item in self.config.get('hla_to_ros', []):
            if 'hla_object_class' not in item: continue
            
            # Check if this update is for the object instance we care about
            if item.get('hla_instance_name') == obj_name:
                # Create ROS message
                msg_type = get_ros_class(item['ros_type'])
                msg = msg_type()
                
                # Map attributes
                class_name = item['hla_object_class']
                for ros_field, hla_attr in item['mapping'].items():
                    if (class_name, hla_attr) in self.hla_manager.attribute_handles:
                        attr_handle = self.hla_manager.attribute_handles[(class_name, hla_attr)]
                        if attributeValues.containsKey(attr_handle):
                            val_bytes = attributeValues.get(attr_handle)
                            # Infer type. Assume float for pose
                            decoded_val = self.hla_manager.decode_value(val_bytes, 'float')
                            set_nested_attr(msg, ros_field, decoded_val)
                
                # Publish
                if item['ros_topic'] in self.pubs:
                    self.pubs[item['ros_topic']].publish(msg)

    def hla_interaction_callback(self, interactionClass, parameterValues):
        # ... (Existing logic) ...
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

    def hla_loop(self):
        if self.hla_manager.is_regulating or self.hla_manager.is_constrained:
            self.hla_manager.advance_time()
        else:
            self.hla_manager.spin_once()

def main(args=None):
    rclpy.init(args=args)
    node = ROS2HLABridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Resign?
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
