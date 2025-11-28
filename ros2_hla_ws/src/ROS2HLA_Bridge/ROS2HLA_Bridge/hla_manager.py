import jpype
import jpype.imports
import sys
import os
import time

# We will delay JVM start until we have the config, or assume it's passed in init.

class HLAManager:
    def __init__(self, config, logger):
        self.config = config
        self.logger = logger
        self.rti_ambassador = None
        self.encoder_factory = None
        self.federate_ambassador = None
        self.joined = False
        
        # Mappings for handles
        self.object_class_handles = {}
        self.attribute_handles = {}
        self.interaction_class_handles = {}
        self.parameter_handles = {}
        self.object_instance_handles = {} # name -> handle (owned/registered by us)
        self.discovered_objects = {} # handle -> name (discovered from others)
        
        # Reservation state
        self.pending_reservation = None
        self.reservation_result = None
        
        # Callbacks
        self.on_interaction_received = None
        self.on_object_update_received = None

        # Time Management
        self.logical_time = 0.0
        self.lookahead = 0.1
        self.time_step = 0.1
        self.is_regulating = False
        self.is_constrained = False
        self.time_advance_granted = False
        self.time_regulation_enabled = False
        self.time_constrained_enabled = False

        # Sync Points
        self.pending_sync_points = []
        self.achieved_sync_points = []

    def start_jvm(self):
        jar_path = self.config['hla']['pitch_jar_path']
        # Try to add prticore.jar explicitly if it exists in the same dir
        jar_dir = os.path.dirname(jar_path)
        core_jar = os.path.join(jar_dir, 'prticore.jar')
        classpath = [jar_path]
        if os.path.exists(core_jar):
            classpath.append(core_jar)
            
        if not jpype.isJVMStarted():
            jpype.startJVM(classpath=classpath)
        
        # Import HLA classes after JVM start
        # Import HLA classes after JVM start
        global RtiFactoryFactory, NullFederateAmbassador, CallbackModel, ResignAction, LogicalTimeFactoryFactory
        from hla.rti1516e import RtiFactoryFactory, NullFederateAmbassador, CallbackModel, ResignAction, LogicalTimeFactoryFactory

    def connect(self):
        self.start_jvm()
        
        rti_factory = RtiFactoryFactory.getRtiFactory()
        self.rti_ambassador = rti_factory.getRtiAmbassador()
        self.encoder_factory = rti_factory.getEncoderFactory()
        
        # Define Ambassador
        @jpype.JImplements("hla.rti1516e.FederateAmbassador")
        class BridgeAmbassador(object):
            def __init__(self, manager):
                self.manager = manager

            @jpype.JOverride
            def receiveInteraction(self, interactionClass, parameterValues, userSuppliedTag, sentOrder, transport, supplementalReceiveInfo=None):
                self.manager._handle_interaction(interactionClass, parameterValues)

            @jpype.JOverride
            def reflectAttributeValues(self, objectInstance, attributeValues, userSuppliedTag, sentOrder, transport, reflectInfo=None):
                self.manager._handle_reflection(objectInstance, attributeValues)

            @jpype.JOverride
            def discoverObjectInstance(self, objectInstance, objectClass, objectName):
                self.manager._handle_discovery(objectInstance, objectName)

            @jpype.JOverride
            def objectInstanceNameReservationSucceeded(self, objectName):
                self.manager._handle_reservation_success(objectName)

            @jpype.JOverride
            def objectInstanceNameReservationFailed(self, objectName):
                self.manager._handle_reservation_failure(objectName)

            # --- Implement all other required methods from FederateAmbassador ---
            @jpype.JOverride
            def announceSynchronizationPoint(self, label, tag, *args):
                self.manager._handle_announce_sync_point(label)

            @jpype.JOverride
            def federationSynchronized(self, label, *args):
                self.manager._handle_federation_synchronized(label)

            @jpype.JOverride
            def timeConstrainedEnabled(self, time, *args):
                self.manager._handle_time_constrained_enabled(time)

            @jpype.JOverride
            def timeRegulationEnabled(self, time, *args):
                self.manager._handle_time_regulation_enabled(time)

            @jpype.JOverride
            def timeAdvanceGrant(self, time, *args):
                self.manager._handle_time_advance_grant(time)

            @jpype.JOverride
            def synchronizationPointRegistrationSucceeded(self, label, *args):
                self.manager._handle_sync_point_registration_succeeded(label)
            @jpype.JOverride
            def synchronizationPointRegistrationFailed(self, label, reason, *args): pass
            @jpype.JOverride
            def provideAttributeValueUpdate(self, *args): pass
            @jpype.JOverride
            def connectionLost(self, reason): print(f"Connection lost: {reason}")
            @jpype.JOverride
            def reportFederationExecutions(self, *args): pass
            @jpype.JOverride
            def initiateFederateSave(self, *args): pass
            @jpype.JOverride
            def federationSaved(self, *args): pass
            @jpype.JOverride
            def federationNotSaved(self, *args): pass
            @jpype.JOverride
            def federationSaveStatusResponse(self, *args): pass
            @jpype.JOverride
            def requestFederationRestoreSucceeded(self, *args): pass
            @jpype.JOverride
            def requestFederationRestoreFailed(self, *args): pass
            @jpype.JOverride
            def federationRestoreBegun(self, *args): pass
            @jpype.JOverride
            def initiateFederateRestore(self, *args): pass
            @jpype.JOverride
            def federationRestored(self, *args): pass
            @jpype.JOverride
            def federationNotRestored(self, *args): pass
            @jpype.JOverride
            def federationRestoreStatusResponse(self, *args): pass
            @jpype.JOverride
            def startRegistrationForObjectClass(self, *args): pass
            @jpype.JOverride
            def stopRegistrationForObjectClass(self, *args): pass
            @jpype.JOverride
            def turnInteractionsOn(self, *args): pass
            @jpype.JOverride
            def turnInteractionsOff(self, *args): pass
            @jpype.JOverride
            def multipleObjectInstanceNameReservationSucceeded(self, *args): pass
            @jpype.JOverride
            def multipleObjectInstanceNameReservationFailed(self, *args): pass
            @jpype.JOverride
            def removeObjectInstance(self, *args): pass
            @jpype.JOverride
            def attributesInScope(self, *args): pass
            @jpype.JOverride
            def attributesOutOfScope(self, *args): pass
            @jpype.JOverride
            def turnUpdatesOnForObjectInstance(self, *args): pass
            @jpype.JOverride
            def turnUpdatesOffForObjectInstance(self, *args): pass
            @jpype.JOverride
            def confirmAttributeTransportationTypeChange(self, *args): pass
            @jpype.JOverride
            def reportAttributeTransportationType(self, *args): pass
            @jpype.JOverride
            def confirmInteractionTransportationTypeChange(self, *args): pass
            @jpype.JOverride
            def reportInteractionTransportationType(self, *args): pass
            @jpype.JOverride
            def requestAttributeOwnershipAssumption(self, *args): pass
            @jpype.JOverride
            def requestDivestitureConfirmation(self, *args): pass
            @jpype.JOverride
            def attributeOwnershipAcquisitionNotification(self, *args): pass
            @jpype.JOverride
            def attributeOwnershipUnavailable(self, *args): pass
            @jpype.JOverride
            def requestAttributeOwnershipRelease(self, *args): pass
            @jpype.JOverride
            def confirmAttributeOwnershipAcquisitionCancellation(self, *args): pass
            @jpype.JOverride
            def informAttributeOwnership(self, *args): pass
            @jpype.JOverride
            def attributeIsNotOwned(self, *args): pass
            @jpype.JOverride
            def attributeIsOwnedByRTI(self, *args): pass
            @jpype.JOverride
            def requestRetraction(self, *args): pass
            @jpype.JOverride
            def warn(self, msg): print(f"WARN: {msg}")

        self.federate_ambassador = BridgeAmbassador(self)
        
        try:
            self.rti_ambassador.connect(self.federate_ambassador, CallbackModel.HLA_EVOKED)
        except Exception as e:
            self.logger.error(f"Error connecting to RTI: {e}")
            raise e

    def _handle_reservation_success(self, objectName):
        if self.pending_reservation == objectName:
            self.reservation_result = True
            self.logger.info(f"Reservation succeeded for {objectName}")

    def _handle_reservation_failure(self, objectName):
        if self.pending_reservation == objectName:
            self.reservation_result = False
            self.logger.warn(f"Reservation failed for {objectName}")

    def _reserve_and_register(self, class_handle, instance_name):
        # 1. Reserve
        self.pending_reservation = instance_name
        self.reservation_result = None
        
        try:
            self.rti_ambassador.reserveObjectInstanceName(instance_name)
        except Exception as e:
            self.logger.warn(f"Exception during reservation request for {instance_name}: {e}")
            # Could be already reserved or other issue. We'll wait and see or try to register.
            # If it throws, maybe we shouldn't wait?
            # If it throws 'IllegalName', we stop.
            # If it throws 'SaveInProgress', etc.
            # Let's assume if it throws we might not get a callback.
            pass

        # 2. Wait for callback
        start_time = time.time()
        while self.reservation_result is None:
            self.rti_ambassador.evokeCallback(0.1)
            if time.time() - start_time > 5.0:
                self.logger.warn(f"Timeout waiting for reservation of {instance_name}")
                break
        
        # 3. Register
        # Even if reservation failed (e.g. because it's already reserved), we try to register.
        # If we own it, it will succeed. If someone else owns it, it will fail.
        try:
            return self.rti_ambassador.registerObjectInstance(class_handle, instance_name)
        except Exception as e:
            self.logger.error(f"Failed to register object {instance_name}: {e}")
            # If we failed to register, maybe we should try to get the handle if it exists?
            # But the requirement implies we are the owner.
            raise e

    def create_and_join_federation(self):
        fed_name = self.config['hla']['federation_name']
        fom_path = self.config['hla']['fom_file_path']
        
        try:
            fom_url = jpype.java.io.File(fom_path).toURI().toURL()
            self.rti_ambassador.createFederationExecution(fed_name, [fom_url])
            self.logger.info(f"Created federation {fed_name}")
        except Exception as e:
            self.logger.warn(f"Federation {fed_name} might already exist: {e}")

        try:
            self.rti_ambassador.joinFederationExecution(
                self.config['hla']['federate_name'],
                "ROS2BridgeType",
                fed_name
            )
            self.joined = True
            self.logger.info("Joined federation")
        except Exception as e:
            self.logger.error(f"Failed to join federation: {e}")
            raise e

    def setup_publications(self, ros_to_hla_config):
        for item in ros_to_hla_config:
            if 'hla_object_class' in item:
                class_name = item['hla_object_class']
                if class_name not in self.object_class_handles:
                    handle = self.rti_ambassador.getObjectClassHandle(class_name)
                    self.object_class_handles[class_name] = handle
                    
                    # Get attribute handles
                    attr_set = self.rti_ambassador.getAttributeHandleSetFactory().create()
                    for _, attr_name in item['mapping'].items():
                        attr_handle = self.rti_ambassador.getAttributeHandle(handle, attr_name)
                        self.attribute_handles[(class_name, attr_name)] = attr_handle
                        attr_set.add(attr_handle)
                    
                    self.rti_ambassador.publishObjectClassAttributes(handle, attr_set)
                    self.logger.info(f"Published Object Class: {class_name}")

                # Register instance if needed
                instance_name = item.get('hla_instance_name')
                if instance_name and instance_name not in self.object_instance_handles:
                    obj_handle = self._reserve_and_register(self.object_class_handles[class_name], instance_name)
                    self.object_instance_handles[instance_name] = obj_handle
                    self.logger.info(f"Registered Object Instance: {instance_name} with handle {obj_handle}")

            elif 'hla_interaction_class' in item:
                int_name = item['hla_interaction_class']
                if int_name not in self.interaction_class_handles:
                    handle = self.rti_ambassador.getInteractionClassHandle(int_name)
                    self.interaction_class_handles[int_name] = handle
                    self.rti_ambassador.publishInteractionClass(handle)
                    self.logger.info(f"Published Interaction Class: {int_name}")
                    
                    # Cache parameter handles from mapping
                    for _, param_name in item['mapping'].items():
                        if (int_name, param_name) not in self.parameter_handles:
                            p_handle = self.rti_ambassador.getParameterHandle(handle, param_name)
                            self.parameter_handles[(int_name, param_name)] = p_handle

                    # Cache parameter handles from fixed_parameters
                    if 'fixed_parameters' in item:
                        for param_name in item['fixed_parameters'].keys():
                            if (int_name, param_name) not in self.parameter_handles:
                                p_handle = self.rti_ambassador.getParameterHandle(handle, param_name)
                                self.parameter_handles[(int_name, param_name)] = p_handle

    def setup_subscriptions(self, hla_to_ros_config):
        # For each config, subscribe to Interaction or Object Class
        for item in hla_to_ros_config:
            if 'hla_interaction_class' in item:
                int_name = item['hla_interaction_class']
                if int_name not in self.interaction_class_handles:
                    try:
                        handle = self.rti_ambassador.getInteractionClassHandle(int_name)
                        self.interaction_class_handles[int_name] = handle
                        self.rti_ambassador.subscribeInteractionClass(handle)
                        self.logger.info(f"Subscribed to Interaction: {int_name}")
                        
                        # Cache parameter handles
                        for _, param_name in item['mapping'].items():
                            p_handle = self.rti_ambassador.getParameterHandle(handle, param_name)
                            self.parameter_handles[(int_name, param_name)] = p_handle
                        
                        # Also cache filter param if exists
                        if 'filter_parameter' in item:
                            fp_name = item['filter_parameter']
                            fp_handle = self.rti_ambassador.getParameterHandle(handle, fp_name)
                            self.parameter_handles[(int_name, fp_name)] = fp_handle
                    except Exception as e:
                        self.logger.error(f"Failed to subscribe to interaction {int_name}: {e}")

            elif 'hla_object_class' in item:
                self.subscribe_object(item['hla_object_class'], item['mapping'].values())

    def subscribe_object(self, class_name, attribute_names):
        if class_name not in self.object_class_handles:
            try:
                handle = self.rti_ambassador.getObjectClassHandle(class_name)
                self.object_class_handles[class_name] = handle
            except Exception as e:
                self.logger.error(f"Failed to get handle for object class {class_name}: {e}")
                return

        class_handle = self.object_class_handles[class_name]
        
        attributes = self.rti_ambassador.getAttributeHandleSetFactory().create()
        for attr in attribute_names:
            if (class_name, attr) not in self.attribute_handles:
                try:
                    attr_handle = self.rti_ambassador.getAttributeHandle(class_handle, attr)
                    self.attribute_handles[(class_name, attr)] = attr_handle
                except Exception as e:
                    self.logger.warn(f"Failed to get handle for attribute {attr} of class {class_name}: {e}")
                    continue
            
            attributes.add(self.attribute_handles[(class_name, attr)])
        
        try:
            self.rti_ambassador.subscribeObjectClassAttributes(class_handle, attributes)
            self.logger.info(f"Subscribed to Object Class: {class_name} with attributes {list(attribute_names)}")
        except Exception as e:
            self.logger.error(f"Failed to subscribe to object class {class_name}: {e}")

    def update_object_attributes(self, instance_name, class_name, data_map):
        # data_map: { attribute_name: value }
        if instance_name not in self.object_instance_handles:
            self.logger.error(f"Unknown instance {instance_name}")
            return

        handle = self.object_instance_handles[instance_name]
        
        # Create AttributeHandleValueMap
        attr_values = self.rti_ambassador.getAttributeHandleValueMapFactory().create(len(data_map))
        
        for attr_name, value in data_map.items():
            if (class_name, attr_name) in self.attribute_handles:
                attr_handle = self.attribute_handles[(class_name, attr_name)]
                
                # Encode value (Assuming float64 for now based on FOM)
                # In a real generic bridge, we need to know the type from FOM or config
                # For this demo, I'll assume float64 for numbers and string for strings
                encoder = None
                if isinstance(value, float):
                    encoder = self.encoder_factory.createHLAfloat64BE()
                    encoder.setValue(value)
                elif isinstance(value, str):
                    encoder = self.encoder_factory.createHLAunicodeString()
                    encoder.setValue(value)
                elif isinstance(value, int):
                    encoder = self.encoder_factory.createHLAfloat64BE() # Treat int as float
                    encoder.setValue(float(value))
                
                if encoder:
                    attr_values.put(attr_handle, encoder.toByteArray())
        
        self.rti_ambassador.updateAttributeValues(handle, attr_values, bytearray())

    def send_interaction(self, interaction_name, data_map):
        if interaction_name not in self.interaction_class_handles:
            self.logger.error(f"Unknown interaction {interaction_name}")
            return

        handle = self.interaction_class_handles[interaction_name]
        
        # Create ParameterHandleValueMap
        param_values = self.rti_ambassador.getParameterHandleValueMapFactory().create(len(data_map))
        
        for param_name, value in data_map.items():
            if (interaction_name, param_name) in self.parameter_handles:
                p_handle = self.parameter_handles[(interaction_name, param_name)]
                
                encoder = None
                if isinstance(value, float):
                    encoder = self.encoder_factory.createHLAfloat64BE()
                    encoder.setValue(value)
                elif isinstance(value, str):
                    encoder = self.encoder_factory.createHLAunicodeString()
                    encoder.setValue(value)
                elif isinstance(value, int):
                    encoder = self.encoder_factory.createHLAfloat64BE()
                    encoder.setValue(float(value))
                
                if encoder:
                    param_values.put(p_handle, encoder.toByteArray())
        
        self.rti_ambassador.sendInteraction(handle, param_values, bytearray())

    def _handle_interaction(self, interactionClass, parameterValues):
        if self.on_interaction_received:
            self.on_interaction_received(interactionClass, parameterValues)

    def _handle_reflection(self, objectInstance, attributeValues):
        if self.on_object_update_received:
            self.on_object_update_received(objectInstance, attributeValues)

    def get_object_name(self, object_instance):
        return self.discovered_objects.get(object_instance)

    def _handle_discovery(self, object_instance, object_name):
        self.logger.info(f"Discovered Object: {object_name} (Handle: {object_instance})")
        self.discovered_objects[object_instance] = object_name



    def spin_once(self):
        if self.rti_ambassador:
            try:
                self.rti_ambassador.evokeCallback(0.01)
            except Exception as e:
                self.logger.warn(f"Error in evokeCallback: {e}")

    def enable_time_management(self):
        tm_config = self.config.get('time_management', {})
        self.is_regulating = tm_config.get('is_regulating', False)
        self.is_constrained = tm_config.get('is_constrained', False)
        self.time_step = tm_config.get('time_step', 1.0)
        self.lookahead = tm_config.get('lookahead', 0.1)
        
        try:
            # Get Time Factory
            self.time_factory = LogicalTimeFactoryFactory.getLogicalTimeFactory("HLAfloat64Time")
            
            if self.is_regulating:
                self.logger.info(f"Enabling Time Regulation with lookahead {self.lookahead}")
                interval = self.time_factory.makeInterval(self.lookahead)
                self.rti_ambassador.enableTimeRegulation(interval)
                
                while not self.time_regulation_enabled:
                    self.rti_ambassador.evokeCallback(0.1)
                    
            if self.is_constrained:
                self.logger.info("Enabling Time Constrained")
                self.rti_ambassador.enableTimeConstrained()
                
                while not self.time_constrained_enabled:
                    self.rti_ambassador.evokeCallback(0.1)
                    
        except Exception as e:
            self.logger.error(f"Error enabling time management: {e}")

        # Initialize Real-Time Pacer
        self.realtime_start_time = time.time()
        self.initial_logical_time = self.logical_time
        self.logger.info(f"Real-Time Pacer initialized. Start Time: {self.realtime_start_time}, Initial Logical Time: {self.initial_logical_time}")

    def advance_time(self, step=None):
        if not (self.is_regulating or self.is_constrained):
            return

        if step is None:
            step = self.time_step

        target_time = self.logical_time + step
        
        # Real-Time Pacer Logic
        # Only pace if we are regulating (driving the time). 
        # If we are only constrained (Client), we should catch up to the leader as fast as possible.
        if hasattr(self, 'realtime_start_time') and self.is_regulating:
            expected_wall_time = target_time - self.initial_logical_time
            current_wall_time = time.time() - self.realtime_start_time
            
            if current_wall_time < expected_wall_time:
                sleep_duration = expected_wall_time - current_wall_time
                # self.logger.info(f"Pacing: Sleeping for {sleep_duration:.4f}s")
                time.sleep(sleep_duration)

        self.time_advance_granted = False
        
        try:
            new_time = self.time_factory.makeTime(target_time)
            self.rti_ambassador.timeAdvanceRequest(new_time)
            
            # Wait for grant
            while not self.time_advance_granted:
                self.rti_ambassador.evokeCallback(0.01)
                
        except Exception as e:
            self.logger.error(f"Error advancing time: {e}")

    def _handle_time_regulation_enabled(self, time):
        self.time_regulation_enabled = True
        self.logical_time = time.getValue()
        self.logger.info(f"Time Regulation Enabled. Current logical time: {self.logical_time}")

    def _handle_time_constrained_enabled(self, time):
        self.time_constrained_enabled = True
        self.logical_time = time.getValue()
        self.logger.info(f"Time Constrained Enabled. Current logical time: {self.logical_time}")

    def _handle_time_advance_grant(self, time):
        self.time_advance_granted = True
        self.logical_time = time.getValue()
        # self.logger.info(f"Time Advance Granted: {self.logical_time}")

    def register_sync_point(self, label):
        try:
            # Check if already exists? RTI throws exception if duplicate.
            # We just try to register.
            self.rti_ambassador.registerFederationSynchronizationPoint(label, bytearray())
            self.logger.info(f"Registering Sync Point: {label}")
            # We wait for success or failure callback
            while label not in self.pending_sync_points and label not in self.achieved_sync_points:
                 self.rti_ambassador.evokeCallback(0.1)
        except Exception as e:
            self.logger.warn(f"Failed to register sync point {label}: {e}")

    def wait_for_sync_point(self, label):
        self.logger.info(f"Waiting for Sync Point: {label}")
        # First ensure we have announced it (received announcement)
        while label not in self.pending_sync_points:
             self.rti_ambassador.evokeCallback(0.1)
        
        # Signal we are ready
        try:
            self.rti_ambassador.synchronizationPointAchieved(label)
        except Exception as e:
             self.logger.warn(f"Error signaling sync point achieved {label}: {e}")

        # Wait for federation synchronized
        while label not in self.achieved_sync_points:
             self.rti_ambassador.evokeCallback(0.1)
        self.logger.info(f"Sync Point Achieved: {label}")

    def _handle_sync_point_registration_succeeded(self, label):
        self.logger.info(f"Sync Point Registration Succeeded: {label}")
        # We don't need to do much, just know it's registered.

    def _handle_announce_sync_point(self, label):
        self.logger.info(f"Sync Point Announced: {label}")
        if label not in self.pending_sync_points:
            self.pending_sync_points.append(label)

    def _handle_federation_synchronized(self, label):
        self.logger.info(f"Federation Synchronized: {label}")
        if label not in self.achieved_sync_points:
            self.achieved_sync_points.append(label)
    def decode_value(self, bytes_val, type_hint):
        # Helper to decode based on expected type
        if type_hint == 'float':
            decoder = self.encoder_factory.createHLAfloat64BE()
            decoder.decode(bytes_val)
            return decoder.getValue()
        elif type_hint == 'string':
            decoder = self.encoder_factory.createHLAunicodeString()
            decoder.decode(bytes_val)
            return decoder.getValue()
        return None
