import sys
import time
import json
import yaml
import importlib
import threading
import jpype
import jpype.imports

# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# HLA Imports (JPype must be started before importing these if not using imports)
PITCH_JAR = "/home/vicen/prti1516e/lib/prti1516e.jar"
if not jpype.isJVMStarted():
    jpype.startJVM(classpath=[PITCH_JAR])

from hla.rti1516e import RtiFactoryFactory, CallbackModel, ResignAction
from hla.rti1516e.exceptions import (
    FederateNotExecutionMember,
    RestoreInProgress,
    SaveInProgress,
    RTIinternalError,
    FederateNameAlreadyInUse,
    ObjectInstanceNameInUse
)

def get_ros_class(type_str):
    """
    Dynamically loads a ROS2 message class from a string.
    e.g., 'geometry_msgs.msg.Twist'
    """
    try:
        parts = type_str.split('.')
        module_name = '.'.join(parts[:-1])
        class_name = parts[-1]
        module = importlib.import_module(module_name)
        return getattr(module, class_name)
    except (ImportError, AttributeError) as e:
        print(f"Error loading ROS type {type_str}: {e}")
        return None

def msg_to_json(msg):
    """
    Converts a ROS2 message to a JSON string.
    This is a simplified conversion. For complex nested types, 
    we might need a recursive approach or use existing tools.
    """
    # Simple approach: use __slots__ or get_fields_and_field_types
    # But for standard msgs, we can often just dump the dict if we convert it.
    # Let's try a recursive dict conversion.
    return json.dumps(msg_to_dict(msg))

def msg_to_dict(msg):
    if hasattr(msg, 'get_fields_and_field_types'):
        d = {}
        for field in msg.get_fields_and_field_types():
            val = getattr(msg, field)
            if hasattr(val, 'get_fields_and_field_types'):
                d[field] = msg_to_dict(val)
            elif hasattr(val, 'tolist'): # Handle numpy arrays / array.array
                d[field] = val.tolist()
            else:
                d[field] = val
        return d
    return msg

def json_to_msg(json_str, msg_type):
    """
    Converts a JSON string to a ROS2 message object.
    """
    try:
        data = json.loads(json_str)
        msg = msg_type()
        populate_msg(msg, data)
        return msg
    except Exception as e:
        print(f"Error parsing JSON to Msg: {e}")
        return None

def populate_msg(msg, data):
    for key, value in data.items():
        if hasattr(msg, key):
            field_val = getattr(msg, key)
            if hasattr(field_val, 'get_fields_and_field_types') and isinstance(value, dict):
                populate_msg(field_val, value)
            else:
                setattr(msg, key, value)

# --- HLA Bridge Class ---

class UniversalBridge(Node):
    def __init__(self, config_path):
        super().__init__('universal_hla_bridge')
        
        # Load Config
        print(f"Loading config from: {config_path}")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        print(f"Loaded Config: {self.config}")
            
        self.federation_name = self.config.get('federation_name', 'RosHlaFed')
        self.federate_name = self.config.get('federate_name', 'UniversalBridge')
        print(f"Federate Name: {self.federate_name}")
        
        self.fom_path = "../generated/universal_fom.xml" # Assumed generated
        
        # ROS2 Setup
        self.publishers_map = {} # topic -> publisher
        self.subscribers_map = {} # topic -> subscriber
        
        # HLA Setup
        self.rti_factory = None
        self.rti_ambassador = None
        self.ambassador = None
        self.object_class_handles = {}
        self.attribute_handles = {}
        self.object_instances = {} # node_name -> instance_handle
        self.known_objects = {} # instance_handle -> node_name (for reflection)
        
        # setup_hla and setup_bridge are now called explicitly
        # self.setup_hla()
        # self.setup_bridge()
        
    def setup_hla(self):
        # Start JVM - Already started at top level
        
        self.RtiFactoryFactory = RtiFactoryFactory
        self.CallbackModel = CallbackModel
        self.ResignAction = ResignAction
        
        self.rti_factory = RtiFactoryFactory.getRtiFactory()
        self.rti_ambassador = self.rti_factory.getRtiAmbassador()
        self.encoder_factory = self.rti_factory.getEncoderFactory()
        
        # Create Ambassador
        self.ambassador = BridgeAmbassador(self)
        
        # Connect
        try:
            self.rti_ambassador.connect(self.ambassador, self.CallbackModel.HLA_EVOKED)
        except Exception:
            pass # Already connected?
            
        # Create Federation Execution
        # Convert path to URL
        from java.io import File
        fom_file = File(self.fom_path)
        fom_url = fom_file.toURI().toURL()

        try:
            self.rti_ambassador.createFederationExecution(self.federation_name, fom_url)
            print(f"Created Federation {self.federation_name}")
        except Exception as e:
            print(f"Federation {self.federation_name} create failed: {e}")
            
        # Join
        try:
            self.rti_ambassador.joinFederationExecution(self.federate_name, "BridgeType", self.federation_name)
            print(f"Joined Federation as {self.federate_name}")
        except FederateNameAlreadyInUse:
            print(f"Federate Name {self.federate_name} already in use.")
            sys.exit(1)

    def setup_bridge(self):
        # Parse Config and Create ROS2/HLA links
        
        for node_name, node_data in self.config.get('nodes', {}).items():
            obj_class_name = node_data.get('object_class', node_name.capitalize())
            
            # Get HLA Handles
            try:
                obj_handle = self.rti_ambassador.getObjectClassHandle(obj_class_name)
                self.object_class_handles[node_name] = obj_handle
            except Exception as e:
                print(f"Error getting handle for {obj_class_name}: {e}")
                continue

            # Identify Attributes to Publish and Subscribe
            pub_attr_set = self.rti_ambassador.getAttributeHandleSetFactory().create()
            sub_attr_set = self.rti_ambassador.getAttributeHandleSetFactory().create()
            
            # Store attribute handles for this object
            if node_name not in self.attribute_handles:
                self.attribute_handles[node_name] = {}

            for attr_name, attr_data in node_data.get('attributes', {}).items():
                try:
                    attr_handle = self.rti_ambassador.getAttributeHandle(obj_handle, attr_name)
                    self.attribute_handles[node_name][attr_name] = attr_handle
                    
                    direction = attr_data.get('direction')
                    if direction == 'ros2_to_hla':
                        pub_attr_set.add(attr_handle)
                    elif direction == 'hla_to_ros2':
                        sub_attr_set.add(attr_handle)
                except Exception as e:
                    print(f"Error getting handle for attribute {attr_name}: {e}")

            # Publish Object Class Attributes (if any)
            if not pub_attr_set.isEmpty():
                try:
                    self.rti_ambassador.publishObjectClassAttributes(obj_handle, pub_attr_set)
                    print(f"Published Attributes for Class {obj_class_name}")
                except Exception as e:
                    print(f"Error publishing attributes for {obj_class_name}: {e}")

            # Subscribe Object Class Attributes (if any)
            if not sub_attr_set.isEmpty():
                try:
                    self.rti_ambassador.subscribeObjectClassAttributes(obj_handle, sub_attr_set)
                    print(f"Subscribed Attributes for Class {obj_class_name}")
                except Exception as e:
                    print(f"Error subscribing attributes for {obj_class_name}: {e}")

            # Reserve Name & Register Object Instance
            self.reserved_names = set()
            self.failed_reservations = set()
            
            try:
                self.rti_ambassador.reserveObjectInstanceName(node_name)
                # Wait for reservation
                start_time = time.time()
                while node_name not in self.reserved_names and node_name not in self.failed_reservations:
                    self.rti_ambassador.evokeCallback(0.1)
                    if time.time() - start_time > 2.0:
                        print(f"Timeout reserving {node_name}, proceeding to register anyway (might fail if not reserved)")
                        break
            except Exception as e:
                print(f"Error reserving {node_name}: {e}")

            try:
                instance_handle = self.rti_ambassador.registerObjectInstance(obj_handle, node_name)
                self.object_instances[node_name] = instance_handle
                self.known_objects[instance_handle] = node_name
                print(f"Registered HLA Object: {node_name}")
            except ObjectInstanceNameInUse:
                print(f"Object {node_name} already exists. Waiting for discovery.")
            except Exception as e:
                print(f"Error registering object {node_name}: {e}")

            
            # Setup ROS Interfaces (regardless of who registered the object)
            for attr_name, attr_data in node_data.get('attributes', {}).items():
                ros_topic = attr_data['ros_topic']
                ros_type_str = attr_data['ros_type']
                direction = attr_data['direction']
                
                ros_msg_type = get_ros_class(ros_type_str)
                if not ros_msg_type:
                    continue
                
                attr_handle = self.attribute_handles[node_name][attr_name]
                
                if direction == 'ros2_to_hla':
                    # ROS2 Sub -> HLA Update
                    # We need to subscribe to ROS2 topic
                    # Callback needs to find instance_handle dynamically
                    cb = self.create_ros_callback(node_name, attr_name, attr_handle)
                    # Use Best Effort QoS to be safe
                    from rclpy.qos import qos_profile_sensor_data
                    sub = self.create_subscription(ros_msg_type, ros_topic, cb, qos_profile_sensor_data)
                    self.subscribers_map[ros_topic] = sub
                    print(f"Mapped ROS2 {ros_topic} -> HLA {node_name}.{attr_name} (Subscribed)")
                    
                elif direction == 'hla_to_ros2':
                    # HLA Reflect -> ROS2 Pub
                    # We need to create a ROS2 Publisher
                    pub = self.create_publisher(ros_msg_type, ros_topic, 10)
                    self.publishers_map[f"{node_name}.{attr_name}"] = pub
                    print(f"Mapped HLA {node_name}.{attr_name} -> ROS2 {ros_topic}")

    def create_ros_callback(self, node_name, attr_name, attr_handle):
        def callback(msg):
            # Look up instance handle
            print(f"Received ROS msg on {node_name}.{attr_name}")
            if node_name not in self.object_instances:
                print(f"Waiting for discovery of {node_name}...")
                return

            instance_handle = self.object_instances[node_name]
            
            json_str = msg_to_json(msg)
            
            # Create AttributeHandleValueMap
            attributes = self.rti_ambassador.getAttributeHandleValueMapFactory().create(1)
            encoder = self.encoder_factory.createHLAunicodeString(json_str)
            attributes.put(attr_handle, encoder.toByteArray())
            
            try:
                if self.is_time_regulated:
                    # Send TSO update
                    # We must send at least at current_time + lookahead
                    # But if we are in the middle of a step, we might need to be careful.
                    # Let's use current_time + lookahead
                    timestamp = self.time_factory.makeTime(self.current_time + self.lookahead)
                    self.rti_ambassador.updateAttributeValues(instance_handle, attributes, None, timestamp)
                    print(f"Updated HLA {node_name}.{attr_name} (TSO: {self.current_time + self.lookahead:.2f})")
                else:
                    self.rti_ambassador.updateAttributeValues(instance_handle, attributes, None)
                    print(f"Updated HLA {node_name}.{attr_name} (RO)")
            except Exception as e:
                print(f"Error updating attribute: {e}")
                pass
        return callback

    def hla_attribute_updated(self, instance_handle, attr_handle, data_bytes):
        # Find which node/attr this is
        node_name = self.known_objects.get(instance_handle)
        if not node_name:
            return # Unknown object
            
        # Find attribute name
        target_attr = None
        if node_name in self.attribute_handles:
            for name, handle in self.attribute_handles[node_name].items():
                if handle.equals(attr_handle):
                    target_attr = name
                    break
        
        if not target_attr:
            return
            
        # Check if we have a publisher for this
        key = f"{node_name}.{target_attr}"
        if key in self.publishers_map:
            # Decode
            decoder = self.encoder_factory.createHLAunicodeString()
            decoder.decode(data_bytes)
            json_str = str(decoder.getValue())
            print(f"Reflect {target_attr} on {node_name} value={json_str}") # Uncommented debug print
            
            # Get Type
            # We need to look up the type again or store it. 
            # Let's store it in publishers_map or similar.
            # For now, look up in config
            type_str = self.config['nodes'][node_name]['attributes'][target_attr]['ros_type']
            msg_type = get_ros_class(type_str)
            
            msg = json_to_msg(json_str, msg_type)
            if msg:
                self.publishers_map[key].publish(msg)
                print(f"Published to ROS2 {key}")

    def setup_time(self):
        self.time_mode = self.config.get('time', {}).get('mode', 'none')
        self.time_step = self.config.get('time', {}).get('step', 0.1)
        self.lookahead = self.config.get('time', {}).get('lookahead', 0.1)
        self.current_time = 0.0
        self.is_time_constrained = False
        self.is_time_regulated = False
        self.publish_clock = self.config.get('time', {}).get('publish_clock', True)
        self.is_time_advance_pending = False
        
        # Get Time Factory
        self.time_factory = self.rti_ambassador.getTimeFactory()

        if self.time_mode == 'hla_master':
            # Bridge regulates time -> Publishes /clock
            print("Enabling Time Regulation (HLA Master)...")
            interval = self.time_factory.makeInterval(self.lookahead)
            self.rti_ambassador.enableTimeRegulation(interval)
            
            # Wait for regulation
            while not self.is_time_regulated:
                self.rti_ambassador.evokeCallback(0.1)
                
            # Also enable constrained to receive updates in order? Usually yes.
            print("Enabling Time Constrained...")
            self.rti_ambassador.enableTimeConstrained()
            while not self.is_time_constrained:
                self.rti_ambassador.evokeCallback(0.1)
                
            # Create Clock Publisher
            from rosgraph_msgs.msg import Clock
            from builtin_interfaces.msg import Time
            self.clock_pub = self.create_publisher(Clock, '/clock', 10)
            print("Time Setup Complete: HLA Master")
            
        elif self.time_mode == 'ros_master':
            # Bridge is constrained AND regulating -> Subscribes to /clock
            print("Enabling Time Regulation & Constrained (ROS Master)...")
            interval = self.time_factory.makeInterval(self.lookahead)
            self.rti_ambassador.enableTimeRegulation(interval)
            while not self.is_time_regulated:
                self.rti_ambassador.evokeCallback(0.1)

            self.rti_ambassador.enableTimeConstrained()
            while not self.is_time_constrained:
                self.rti_ambassador.evokeCallback(0.1)
                
            # Subscribe to /clock
            from rosgraph_msgs.msg import Clock
            self.create_subscription(Clock, '/clock', self.ros_clock_callback, 10)
            print("Time Setup Complete: ROS Master")

        elif self.time_mode == 'hla_slave':
            # Bridge is constrained -> Publishes /clock from HLA time
            print("Enabling Time Constrained (HLA Slave)...")
            self.rti_ambassador.enableTimeConstrained()
            while not self.is_time_constrained:
                try:
                    self.rti_ambassador.evokeCallback(0.1)
                except Exception as e:
                    print(f"Error in setup_time evokeCallback: {e}")
            
            # Create Clock Publisher
            if self.publish_clock:
                from rosgraph_msgs.msg import Clock
                from builtin_interfaces.msg import Time
                self.clock_pub = self.create_publisher(Clock, '/clock', 10)
            print("Time Setup Complete: HLA Slave")

    def ros_clock_callback(self, msg):
        # ROS2 Time drives HLA Time
        # We request TAR to msg.clock
        ros_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        
        # Debug: Print clock every second roughly
        if int(ros_time) > int(self.current_time):
             print(f"Clock: {ros_time:.2f}, Current: {self.current_time:.2f}, Pending: {self.is_time_advance_pending}")

        # Only advance if ros_time > current_time + step AND no pending request
        # This prevents spamming TARs for every microsecond update
        if ros_time >= self.current_time + self.time_step and not self.is_time_advance_pending:
            # Align to next step
            target_time = self.current_time + self.time_step
            hla_time = self.time_factory.makeTime(target_time)
            try:
                self.rti_ambassador.timeAdvanceRequest(hla_time)
                self.is_time_advance_pending = True
                # print(f"Requested TAR: {target_time}")
            except Exception as e:

                print(f"Error requesting TAR: {e}")
                pass

    def diagnostic_timer_callback(self):
        count = self.count_publishers('/robot_1/cmd_vel')
        print(f"DEBUG: Publishers on /robot_1/cmd_vel: {count}")

    def run(self):
        print("Bridge Running...")
        # Debug: Hardcoded subscription
        from geometry_msgs.msg import Twist
        from rclpy.qos import qos_profile_sensor_data
        self.debug_sub = self.create_subscription(
            Twist, 
            '/robot_1/cmd_vel', 
            lambda msg: print(f"DEBUG: Hardcoded sub received Twist: {msg.linear.x}"), 
            qos_profile_sensor_data
        )
        print(f"DEBUG: Created hardcoded subscription on /robot_1/cmd_vel. Node: {self.get_name()}, NS: {self.get_namespace()}")
        
        # Debug: Diagnostic Timer
        self.create_timer(2.0, self.diagnostic_timer_callback)

        # self.setup_time()
        
        print("Entering run loop...")
        try:
            while rclpy.ok():
                # print("Loop start")
                try:
                    rclpy.spin_once(self, timeout_sec=0.001)
                except Exception as e:
                    print(f"Error in spin_once: {e}")
                
                # print("Pre evoke")
                # try:
                #     self.rti_ambassador.evokeCallback(0.001)
                # except Exception as e:
                #     print(f"Error in evokeCallback: {e}")
                #     import traceback
                #     traceback.print_exc()
                # except BaseException as e:
                #     print(f"BaseException in evokeCallback: {e}")
                #     import traceback
                #     traceback.print_exc()
                # print("Post evoke")

                # If we are HLA Slave, we drive the time by requesting TARs
                # if self.time_mode == 'hla_slave' and self.is_time_constrained and not self.is_time_advance_pending:
                #     target_time = self.current_time + self.time_step
                #     hla_time = self.time_factory.makeTime(target_time)
                #     try:
                #         self.rti_ambassador.timeAdvanceRequest(hla_time)
                #         self.is_time_advance_pending = True
                #         # print(f"Requested TAR (Slave): {target_time:.2f}")
                #     except Exception as e:
                #         print(f"Error requesting TAR (Slave): {e}")
                #         pass
                
                # if self.time_mode == 'hla_master':
                #     # Drive the simulation
                #     # Request next step
                #     next_time = self.current_time + self.time_step
                #     hla_next_time = self.time_factory.makeTime(next_time)
                pass
                    
                    try:
                        self.rti_ambassador.timeAdvanceRequest(hla_next_time)
                        
                        # Wait for grant (blocking spin?)
                        # We shouldn't block completely, but we can't proceed until granted.
                        # In a real loop we might just return and check grant status.
                        # For simplicity, we loop here waiting for grant OR ros callbacks
                        while self.current_time < next_time:
                             self.rti_ambassador.evokeCallback(0.001)
                             rclpy.spin_once(self, timeout_sec=0.001)
                             
                        # Publish Clock
                        from rosgraph_msgs.msg import Clock
                        from builtin_interfaces.msg import Time
                        msg = Clock()
                        sec = int(self.current_time)
                        nanosec = int((self.current_time - sec) * 1e9)
                        msg.clock = Time(sec=sec, nanosec=nanosec)
                        self.clock_pub.publish(msg)
                        
                    except Exception as e:
                        # print(f"TAR Error: {e}")
                        pass

                elif self.time_mode == 'hla_slave':
                    # Just follow HLA time
                    # We need to request time advance to receive grants
                    # In slave mode, we usually just want to go as fast as possible or step by step?
                    # If we want to sync with HLA, we should request Next Time.
                    
                    next_time = self.current_time + self.time_step
                    hla_next_time = self.time_factory.makeTime(next_time)
                    
                    try:
                        self.rti_ambassador.timeAdvanceRequest(hla_next_time)
                        # Wait for grant
                        while self.current_time < next_time:
                             self.rti_ambassador.evokeCallback(0.001)
                             rclpy.spin_once(self, timeout_sec=0.001)
                        
                        # Publish Clock
                        from rosgraph_msgs.msg import Clock
                        from builtin_interfaces.msg import Time
                        msg = Clock()
                        sec = int(self.current_time)
                        nanosec = int((self.current_time - sec) * 1e9)
                        msg.clock = Time(sec=sec, nanosec=nanosec)
                        self.clock_pub.publish(msg)
                        
                    except Exception as e:
                        pass

        except KeyboardInterrupt:
            pass
        finally:
            try:
                self.rti_ambassador.resignFederationExecution(self.ResignAction.DELETE_OBJECTS)
                print("Resigned Federation")
            except Exception as e:
                print(f"Resign failed: {e}")
            
            try:
                self.rti_ambassador.destroyFederationExecution(self.federation_name)
                print(f"Destroyed Federation {self.federation_name}")
            except Exception as e:
                print(f"Destroy failed (maybe others are still joined?): {e}")

    # Time Callbacks
    @jpype.JOverride
    def timeRegulationEnabled(self, time, *args):
        self.current_time = time.getValue()
        self.is_time_regulated = True
        print(f"Time Regulation Enabled. Time: {self.current_time}")

    @jpype.JOverride
    def timeConstrainedEnabled(self, time, *args):
        self.current_time = time.getValue()
        self.is_time_constrained = True
        print(f"Time Constrained Enabled. Time: {self.current_time}")

    @jpype.JOverride
    def timeAdvanceGrant(self, time, *args):
        self.current_time = time.getValue()
        self.is_time_advance_pending = False
        print(f"Time Granted: {self.current_time:.2f}")
        if self.time_mode == 'hla_slave':
             # Optional: Publish /clock if configured
             pass

    @jpype.JOverride
    def discoverObjectInstance(self, instance_handle, object_class_handle, object_name, *args):
        print(f"Discovered Object: {object_name} (Handle: {instance_handle})")
        self.object_instances[object_name] = instance_handle
        self.known_objects[instance_handle] = object_name
        
        # We might need to subscribe to attributes if we are interested in this object
        # Check config
        if object_name in self.config.get('nodes', {}):
            node_data = self.config['nodes'][object_name]
            
            # Subscribe to attributes
            sub_attr_set = self.rti_ambassador.getAttributeHandleSetFactory().create()
            # Acquire ownership for attributes we want to update
            acquire_attr_set = self.rti_ambassador.getAttributeHandleSetFactory().create()
            
            for attr_name, attr_data in node_data.get('attributes', {}).items():
                if attr_name in self.attribute_handles.get(object_name, {}):
                    attr_handle = self.attribute_handles[object_name][attr_name]
                    
                    if attr_data['direction'] == 'hla_to_ros2':
                         # We want to receive this from HLA
                         sub_attr_set.add(attr_handle)
                    elif attr_data['direction'] == 'ros2_to_hla':
                         # We want to update this, so we need ownership
                         acquire_attr_set.add(attr_handle)
            
            if not sub_attr_set.isEmpty():
                try:
                    self.rti_ambassador.subscribeObjectClassAttributes(object_class_handle, sub_attr_set)
                    print(f"Subscribed to attributes for discovered object {object_name}")
                except Exception as e:
                    print(f"Error subscribing attributes: {e}")
                    
            if not acquire_attr_set.isEmpty():
                try:
                    self.rti_ambassador.attributeOwnershipAcquisition(instance_handle, acquire_attr_set, None)
                    print(f"Requested negotiated ownership acquisition for attributes of {object_name}")
                except Exception as e:
                    print(f"Error requesting ownership acquisition: {e}")



    # Ownership Callbacks
    def requestAttributeOwnershipRelease(self, instance_handle, attr_handles, tag):
        # Always agree to release ownership
        try:
            self.rti_ambassador.attributeOwnershipDivestitureIfWanted(instance_handle, attr_handles)
            print(f"Released ownership of attributes for instance {instance_handle}")
        except Exception as e:
            print(f"Error releasing ownership: {e}")

    def attributeOwnershipAcquisitionNotification(self, instance_handle, attr_handles, tag):
        print(f"Acquired ownership of attributes for instance {instance_handle}")

    def attributeOwnershipUnavailable(self, instance_handle, attr_handles):
        print(f"Ownership unavailable for instance {instance_handle}")

# Import Ambassador
from bridge_hla2ros2.bridge_ambassador import BridgeAmbassador

def main():
    import sys
    import argparse
    import os
    from ament_index_python.packages import get_package_share_directory
    
    parser = argparse.ArgumentParser(description='Universal ROS2-HLA Bridge')
    parser.add_argument('config', nargs='?', default=None, help='Path to configuration YAML')
    # Use parse_known_args to ignore ROS2 specific arguments like --ros-args
    args, unknown = parser.parse_known_args()
    
    # If config not provided, try to find default in share (not recommended for this usage but good fallback)
    if args.config:
        config_path = args.config
    else:
        # Default fallback
        try:
            share_dir = get_package_share_directory('bridge_hla2ros2')
            config_path = os.path.join(share_dir, 'config', 'bridge_config.yaml')
        except Exception:
            print("Config not provided and package share not found.")
            sys.exit(1)

    # Resolve FOM path using ament_index
    try:
        share_dir = get_package_share_directory('bridge_hla2ros2')
        fom_path = os.path.join(share_dir, 'generated', 'universal_fom.xml')
    except Exception:
        # Fallback for local development if not installed
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        fom_path = os.path.join(project_root, 'generated', 'universal_fom.xml')
    
    # Ensure generated dir exists (if writing to it, but usually we install it read-only?)
    # Wait, if we generate FOM, we need write access.
    # If installed in /opt/ros or install space, we might not have write access.
    # The FOM generator should probably write to a temp dir or the user should provide a path.
    # For now, let's assume we write to the same location as the config or a temp location.
    # Actually, the FOM is static for the bridge? Or generated from config?
    # It is generated from config. So we should generate it to a temp path or /tmp.
    
    import tempfile
    fom_path = os.path.join(tempfile.gettempdir(), 'universal_fom.xml')
    
    # Generate FOM first
    import bridge_hla2ros2.fom_generator as fom_generator
    fom_generator.generate_fom(config_path, fom_path)
    
    rclpy.init()
    bridge = UniversalBridge(config_path)
    bridge.fom_path = fom_path # Override default
    
    # Perform setup now that configuration is complete
    bridge.setup_hla()
    bridge.setup_bridge()
    
    bridge.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
