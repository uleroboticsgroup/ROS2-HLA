"""
HLA Manager para el sistema DualSim.
Gestiona la conexión a la RTI, publicación/suscripción de objetos,
y sincronización temporal (Time Regulating + Time Constrained).

Extiende el patrón del ROS2HLA_Bridge original con soporte para
codificación float32LE (compatible con el plugin C++ de Unity).
"""

import jpype
import jpype.imports
import os
import time
import struct
import threading


class HLAManager:
    def __init__(self, config, logger):
        self.config = config
        self.logger = logger
        self.rti_ambassador = None
        self.encoder_factory = None
        self.federate_ambassador = None
        self.joined = False

        # Handles
        self.object_class_handles = {}
        self.attribute_handles = {}
        self.object_instance_handles = {}   # name -> handle (objetos propios)
        self.discovered_objects = {}         # handle -> name (objetos remotos)

        # Callbacks
        self.on_object_update_received = None

        # Reservación de nombres
        self.pending_reservation = None
        self.reservation_result = None

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

        # Lock para thread-safety
        self.lock = threading.Lock()

    def start_jvm(self):
        jar_path = self.config['hla']['pitch_jar_path']
        jar_dir = os.path.dirname(jar_path)
        core_jar = os.path.join(jar_dir, 'prticore.jar')
        classpath = [jar_path]
        if os.path.exists(core_jar):
            classpath.append(core_jar)

        if not jpype.isJVMStarted():
            jpype.startJVM(classpath=classpath)

        global RtiFactoryFactory, CallbackModel, ResignAction, LogicalTimeFactoryFactory
        from hla.rti1516e import (
            RtiFactoryFactory, CallbackModel, ResignAction,
            LogicalTimeFactoryFactory
        )

    def connect(self):
        self.start_jvm()

        rti_factory = RtiFactoryFactory.getRtiFactory()
        self.rti_ambassador = rti_factory.getRtiAmbassador()
        self.encoder_factory = rti_factory.getEncoderFactory()

        manager = self

        @jpype.JImplements("hla.rti1516e.FederateAmbassador")
        class DualSimAmbassador(object):
            def __init__(self):
                pass

            @jpype.JOverride
            def reflectAttributeValues(self, objectInstance, attributeValues,
                                       userSuppliedTag, sentOrder, transport,
                                       reflectInfo=None):
                try:
                    manager._handle_reflection(objectInstance, attributeValues)
                except Exception as e:
                    print(f"Error in reflectAttributeValues: {e}")

            @jpype.JOverride
            def discoverObjectInstance(self, objectInstance, objectClass,
                                       objectName):
                try:
                    manager._handle_discovery(objectInstance, objectName)
                except Exception as e:
                    print(f"Error in discoverObjectInstance: {e}")

            @jpype.JOverride
            def objectInstanceNameReservationSucceeded(self, objectName):
                try:
                    manager._handle_reservation_success(objectName)
                except Exception as e:
                    print(f"Error in reservation success: {e}")

            @jpype.JOverride
            def objectInstanceNameReservationFailed(self, objectName):
                try:
                    manager._handle_reservation_failure(objectName)
                except Exception as e:
                    print(f"Error in reservation failure: {e}")

            @jpype.JOverride
            def announceSynchronizationPoint(self, label, tag, *args):
                try:
                    manager._handle_announce_sync_point(label)
                except Exception as e:
                    print(f"Error in announceSynchronizationPoint: {e}")

            @jpype.JOverride
            def federationSynchronized(self, label, *args):
                try:
                    manager._handle_federation_synchronized(label)
                except Exception as e:
                    print(f"Error in federationSynchronized: {e}")

            @jpype.JOverride
            def synchronizationPointRegistrationSucceeded(self, label, *args):
                manager.logger.info(
                    f"Sync Point Registration Succeeded: {label}"
                )

            @jpype.JOverride
            def timeConstrainedEnabled(self, time_val, *args):
                try:
                    manager._handle_time_constrained_enabled(time_val)
                except Exception as e:
                    print(f"Error in timeConstrainedEnabled: {e}")

            @jpype.JOverride
            def timeRegulationEnabled(self, time_val, *args):
                try:
                    manager._handle_time_regulation_enabled(time_val)
                except Exception as e:
                    print(f"Error in timeRegulationEnabled: {e}")

            @jpype.JOverride
            def timeAdvanceGrant(self, time_val, *args):
                try:
                    manager._handle_time_advance_grant(time_val)
                except Exception as e:
                    print(f"Error in timeAdvanceGrant: {e}")

            # --- Callbacks no usados ---
            @jpype.JOverride
            def synchronizationPointRegistrationFailed(self, *args): pass
            @jpype.JOverride
            def provideAttributeValueUpdate(self, *args): pass
            @jpype.JOverride
            def receiveInteraction(self, *args): pass
            @jpype.JOverride
            def connectionLost(self, reason):
                print(f"Conexión perdida: {reason}")
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
            def multipleObjectInstanceNameReservationSucceeded(self, *args):
                pass
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
            def confirmAttributeOwnershipAcquisitionCancellation(self, *args):
                pass
            @jpype.JOverride
            def informAttributeOwnership(self, *args): pass
            @jpype.JOverride
            def attributeIsNotOwned(self, *args): pass
            @jpype.JOverride
            def attributeIsOwnedByRTI(self, *args): pass
            @jpype.JOverride
            def requestRetraction(self, *args): pass

        self.federate_ambassador = DualSimAmbassador()

        try:
            self.rti_ambassador.connect(
                self.federate_ambassador, CallbackModel.HLA_EVOKED
            )
            self.logger.info("Conectado a la RTI")
        except Exception as e:
            self.logger.error(f"Error conectando a la RTI: {e}")
            raise e

    def create_and_join_federation(self):
        fed_name = self.config['hla']['federation_name']
        fom_path = self.config['hla']['fom_file_path']

        try:
            fom_url = jpype.java.io.File(fom_path).toURI().toURL()
            self.rti_ambassador.createFederationExecution(fed_name, [fom_url])
            self.logger.info(f"Federación '{fed_name}' creada")
        except Exception as e:
            self.logger.warn(
                f"Federación '{fed_name}' ya existe o error al crear: {e}"
            )

        try:
            self.rti_ambassador.joinFederationExecution(
                self.config['hla']['federate_name'],
                "DualSimBridgeType",
                fed_name
            )
            self.joined = True
            self.logger.info(
                f"Unido a federación '{fed_name}' como "
                f"'{self.config['hla']['federate_name']}'"
            )
        except Exception as e:
            self.logger.error(f"Error al unirse a la federación: {e}")
            raise e

    # ─── Publicación (objetos propios) ───────────────────────────

    def setup_publication(self, class_name, attribute_names, instance_name):
        """Publica una clase de objeto HLA y registra una instancia."""
        if class_name not in self.object_class_handles:
            handle = self.rti_ambassador.getObjectClassHandle(class_name)
            self.object_class_handles[class_name] = handle

            attr_set = (
                self.rti_ambassador.getAttributeHandleSetFactory().create()
            )
            for attr_name in attribute_names:
                attr_h = self.rti_ambassador.getAttributeHandle(
                    handle, attr_name
                )
                self.attribute_handles[(class_name, attr_name)] = attr_h
                attr_set.add(attr_h)

            self.rti_ambassador.publishObjectClassAttributes(handle, attr_set)
            self.logger.info(
                f"Publicando clase '{class_name}' "
                f"con atributos {attribute_names}"
            )

        # Reservar y registrar instancia
        if instance_name not in self.object_instance_handles:
            obj_handle = self._reserve_and_register(
                self.object_class_handles[class_name], instance_name
            )
            self.object_instance_handles[instance_name] = obj_handle
            self.logger.info(
                f"Instancia '{instance_name}' registrada "
                f"(handle: {obj_handle})"
            )

    def _reserve_and_register(self, class_handle, instance_name):
        self.pending_reservation = instance_name
        self.reservation_result = None

        try:
            self.rti_ambassador.reserveObjectInstanceName(instance_name)
        except Exception as e:
            self.logger.warn(f"Error reservando '{instance_name}': {e}")

        start = time.time()
        while self.reservation_result is None:
            self.rti_ambassador.evokeCallback(0.1)
            if time.time() - start > 5.0:
                self.logger.warn(
                    f"Timeout esperando reserva de '{instance_name}'"
                )
                break

        try:
            return self.rti_ambassador.registerObjectInstance(
                class_handle, instance_name
            )
        except Exception as e:
            err_msg = str(e)
            if 'already registered' in err_msg.lower():
                self.logger.warn(
                    f"Instancia '{instance_name}' ya existe en RTI. "
                    f"Registrando sin nombre específico..."
                )
                try:
                    return self.rti_ambassador.registerObjectInstance(
                        class_handle
                    )
                except Exception as e2:
                    self.logger.error(
                        f"Error al registrar instancia sin nombre: {e2}"
                    )
                    raise e2
            else:
                self.logger.error(
                    f"Error al registrar instancia '{instance_name}': {e}"
                )
                raise e

    def _handle_reservation_success(self, objectName):
        if self.pending_reservation == objectName:
            self.reservation_result = True
            self.logger.info(f"Reserva exitosa para '{objectName}'")

    def _handle_reservation_failure(self, objectName):
        if self.pending_reservation == objectName:
            self.reservation_result = False
            self.logger.warn(f"Reserva fallida para '{objectName}'")

    # ─── Suscripción (objetos remotos) ───────────────────────────

    def setup_subscription(self, class_name, attribute_names):
        """Se suscribe a una clase de objeto HLA para recibir actualizaciones."""
        if class_name not in self.object_class_handles:
            handle = self.rti_ambassador.getObjectClassHandle(class_name)
            self.object_class_handles[class_name] = handle
        else:
            handle = self.object_class_handles[class_name]

        attr_set = self.rti_ambassador.getAttributeHandleSetFactory().create()
        for attr_name in attribute_names:
            if (class_name, attr_name) not in self.attribute_handles:
                attr_h = self.rti_ambassador.getAttributeHandle(
                    handle, attr_name
                )
                self.attribute_handles[(class_name, attr_name)] = attr_h
            attr_set.add(self.attribute_handles[(class_name, attr_name)])

        self.rti_ambassador.subscribeObjectClassAttributes(handle, attr_set)
        self.logger.info(
            f"Suscrito a clase '{class_name}' "
            f"con atributos {attribute_names}"
        )

    # ─── Actualización de atributos ──────────────────────────────

    def update_object(self, instance_name, class_name, data_map, encoding='float32le'):
        """
        Actualiza los atributos de un objeto registrado.
        data_map: {attr_name: float_value}
        encoding: 'float32le' (Unity compatible) o 'float64be'
        """
        if instance_name not in self.object_instance_handles:
            self.logger.error(f"Instancia desconocida: '{instance_name}'")
            return

        handle = self.object_instance_handles[instance_name]
        attr_values = (
            self.rti_ambassador.getAttributeHandleValueMapFactory()
            .create(len(data_map))
        )

        for attr_name, value in data_map.items():
            key = (class_name, attr_name)
            if key in self.attribute_handles:
                attr_handle = self.attribute_handles[key]
                encoded = self._encode_float(float(value), encoding)
                attr_values.put(attr_handle, encoded)

        self.rti_ambassador.updateAttributeValues(
            handle, attr_values, b""
        )

    def _encode_float(self, value, encoding):
        """Codifica un float según el formato especificado."""
        if encoding == 'float32le':
            # Pack como float32 little-endian (4 bytes) en JArray para Java
            raw_bytes = struct.pack('<f', value)
            return jpype.JArray(jpype.JByte)(raw_bytes)
        elif encoding == 'float64be':
            encoder = self.encoder_factory.createHLAfloat64BE()
            encoder.setValue(value)
            return encoder.toByteArray()
        else:
            raise ValueError(f"Encoding desconocido: {encoding}")

    def _decode_float(self, byte_data, encoding):
        """Decodifica bytes a float según el formato especificado."""
        if encoding == 'float32le':
            # Convertir de Java byte[] a bytes Python
            raw = bytes(byte_data)
            return struct.unpack('<f', raw)[0]
        elif encoding == 'float64be':
            decoder = self.encoder_factory.createHLAfloat64BE()
            decoder.decode(byte_data)
            return decoder.getValue()
        else:
            raise ValueError(f"Encoding desconocido: {encoding}")

    # ─── Callbacks de objetos remotos ────────────────────────────

    def _handle_reflection(self, objectInstance, attributeValues):
        if self.on_object_update_received:
            self.on_object_update_received(objectInstance, attributeValues)

    def _handle_discovery(self, objectInstance, objectName):
        self.logger.info(
            f"Objeto descubierto: '{objectName}' (handle: {objectInstance})"
        )
        self.discovered_objects[objectInstance] = objectName

    def get_object_name(self, object_instance):
        return self.discovered_objects.get(object_instance)

    # ─── Time Management ─────────────────────────────────────────

    def enable_time_management(self):
        tm_config = self.config.get('time_management', {})
        self.is_regulating = tm_config.get('is_regulating', False)
        self.is_constrained = tm_config.get('is_constrained', False)
        self.time_step = tm_config.get('time_step', 0.1)
        self.lookahead = tm_config.get('lookahead', 0.1)

        try:
            self.time_factory = LogicalTimeFactoryFactory.getLogicalTimeFactory(
                "HLAfloat64Time"
            )

            if self.is_regulating:
                self.logger.info(
                    f"Activando Time Regulation (lookahead={self.lookahead})"
                )
                interval = self.time_factory.makeInterval(self.lookahead)
                self.rti_ambassador.enableTimeRegulation(interval)
                while not self.time_regulation_enabled:
                    self.rti_ambassador.evokeCallback(0.1)

            if self.is_constrained:
                self.logger.info("Activando Time Constrained")
                self.rti_ambassador.enableTimeConstrained()
                while not self.time_constrained_enabled:
                    self.rti_ambassador.evokeCallback(0.1)

        except Exception as e:
            self.logger.error(
                f"Error activando gestión de tiempo: {e}"
            )

        # Real-Time Pacer
        self.realtime_start_time = time.time()
        self.initial_logical_time = self.logical_time
        self.logger.info(
            f"Pacer inicializado. Tiempo lógico inicial: "
            f"{self.initial_logical_time}"
        )

    def advance_time(self, step=None):
        if not (self.is_regulating or self.is_constrained):
            return

        if step is None:
            step = self.time_step

        target_time = self.logical_time + step

        # Real-Time Pacer: dormir si vamos demasiado rápido
        if hasattr(self, 'realtime_start_time') and self.is_regulating:
            expected_wall = target_time - self.initial_logical_time
            current_wall = time.time() - self.realtime_start_time
            if current_wall < expected_wall:
                time.sleep(expected_wall - current_wall)

        self.time_advance_granted = False

        try:
            new_time = self.time_factory.makeTime(target_time)
            self.rti_ambassador.timeAdvanceRequest(new_time)

            while not self.time_advance_granted:
                self.rti_ambassador.evokeCallback(0.01)

        except Exception as e:
            self.logger.error(f"Error avanzando tiempo: {e}")

    def spin_once(self):
        if self.rti_ambassador:
            try:
                self.rti_ambassador.evokeCallback(0.01)
            except Exception as e:
                self.logger.warn(f"Error en evokeCallback: {e}")

    def _handle_time_regulation_enabled(self, time_val):
        self.time_regulation_enabled = True
        self.logical_time = time_val.getValue()
        self.logger.info(
            f"Time Regulation activada. Tiempo: {self.logical_time}"
        )

    def _handle_time_constrained_enabled(self, time_val):
        self.time_constrained_enabled = True
        self.logical_time = time_val.getValue()
        self.logger.info(
            f"Time Constrained activado. Tiempo: {self.logical_time}"
        )

    def _handle_time_advance_grant(self, time_val):
        self.time_advance_granted = True
        self.logical_time = time_val.getValue()

    # ─── Sync Points ─────────────────────────────────────────────

    def register_sync_point(self, label):
        try:
            self.rti_ambassador.registerFederationSynchronizationPoint(
                label, bytearray()
            )
            self.logger.info(f"Registrando Sync Point: '{label}'")
            while (label not in self.pending_sync_points
                   and label not in self.achieved_sync_points):
                self.rti_ambassador.evokeCallback(0.1)
        except Exception as e:
            self.logger.warn(
                f"Error al registrar sync point '{label}': {e}"
            )

    def wait_for_sync_point(self, label):
        self.logger.info(f"Esperando Sync Point: '{label}'")
        while label not in self.pending_sync_points:
            self.rti_ambassador.evokeCallback(0.1)

        try:
            self.rti_ambassador.synchronizationPointAchieved(label)
        except Exception as e:
            self.logger.warn(
                f"Error señalando sync point achieved '{label}': {e}"
            )

        while label not in self.achieved_sync_points:
            self.rti_ambassador.evokeCallback(0.1)
        self.logger.info(f"Sync Point '{label}' alcanzado")

    def _handle_announce_sync_point(self, label):
        self.logger.info(f"Sync Point anunciado: '{label}'")
        if label not in self.pending_sync_points:
            self.pending_sync_points.append(label)

    def _handle_federation_synchronized(self, label):
        self.logger.info(f"Federación sincronizada: '{label}'")
        if label not in self.achieved_sync_points:
            self.achieved_sync_points.append(label)

    # ─── Cleanup ─────────────────────────────────────────────────

    def disconnect(self):
        if not self.joined:
            return
        try:
            self.rti_ambassador.resignFederationExecution(
                ResignAction.DELETE_OBJECTS
            )
            self.logger.info("Desconectado de la federación")
        except Exception as e:
            self.logger.warn(f"Error al desconectarse: {e}")
        try:
            self.rti_ambassador.destroyFederationExecution(
                self.config['hla']['federation_name']
            )
            self.logger.info("Federación destruida")
        except Exception:
            pass  # Otros federados aún conectados
