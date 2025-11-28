import jpype
import jpype.imports
import time
import math
import sys

# --- CONFIGURATION ---
PITCH_JAR = "/home/vicen/prti1516e/lib/prti1516e.jar"
jpype.startJVM(classpath=[PITCH_JAR])

from hla.rti1516e import RtiFactoryFactory, CallbackModel, ResignAction

# --- PUBLISHER CLASS ---
# Implements the FederateAmbassador interface to receive callbacks from the RTI
@jpype.JImplements("hla.rti1516e.FederateAmbassador")
class PublisherAmbassador(object):
    def __init__(self):
        self.is_synchronized = False
        self.is_time_regulating = False
        self.current_time = 0.0

    @jpype.JOverride
    def synchronizationPointRegistrationSucceeded(self, label, *args):
        print(f"Sync Point Registrado: {label}")

    @jpype.JOverride
    def synchronizationPointRegistrationFailed(self, label, reason, *args):
        print(f"Fallo registro Sync Point: {label} ({reason})")

    @jpype.JOverride
    def announceSynchronizationPoint(self, label, tag, *args):
        print(f"Sync Point Anunciado: {label}")

    @jpype.JOverride
    def federationSynchronized(self, label, *args):
        print(f"Federación Sincronizada: {label}")
        self.is_synchronized = True

    @jpype.JOverride
    def timeRegulationEnabled(self, time, *args):
        self.current_time = time.getValue()
        self.is_time_regulating = True
        print(f"Time Regulation Enabled. Current Time: {self.current_time}")

    @jpype.JOverride
    def timeAdvanceGrant(self, time, *args):
        self.current_time = time.getValue()
        
    # --- Empty implementations for unused callbacks ---
    @jpype.JOverride
    def discoverObjectInstance(self, *args): pass
    @jpype.JOverride
    def provideAttributeValueUpdate(self, *args): pass
    @jpype.JOverride
    def reflectAttributeValues(self, *args): pass
    @jpype.JOverride
    def connectionLost(self, *args): pass
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
    def objectInstanceNameReservationSucceeded(self, *args): pass
    @jpype.JOverride
    def objectInstanceNameReservationFailed(self, *args): pass
    @jpype.JOverride
    def multipleObjectInstanceNameReservationSucceeded(self, *args): pass
    @jpype.JOverride
    def multipleObjectInstanceNameReservationFailed(self, *args): pass
    @jpype.JOverride
    def receiveInteraction(self, *args): pass
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
    def timeConstrainedEnabled(self, *args): pass
    @jpype.JOverride
    def requestRetraction(self, *args): pass


def run_publisher():
    rti_factory = RtiFactoryFactory.getRtiFactory()
    rti_ambassador = rti_factory.getRtiAmbassador()
    encoder_factory = rti_factory.getEncoderFactory()
    
    ambassador = PublisherAmbassador()
    
    # 1. Connect
    try:
        rti_ambassador.connect(ambassador, CallbackModel.HLA_EVOKED)
    except Exception as e:
        print(f"Error connecting: {e}")
        return

    # 2. Create/Join Federation
    try:
        fom_url = jpype.java.io.File("/home/vicen/ISDEFE/Pruebas_timers_HLA/sine.xml").toURI().toURL()
        rti_ambassador.createFederationExecution("SineFed", [fom_url])
        print("Federation created.")
    except Exception:
        print("Federation already exists (or error).")

    rti_ambassador.joinFederationExecution("SinePublisher", "SinePublisherType", "SineFed")
    print("Joined federation as SinePublisher.")
    
    # Get Time Factory AFTER joining (required by some RTIs)
    time_factory = rti_ambassador.getTimeFactory()

    # 3. Enable Time Regulation
    # Lookahead: Minimum time distance to future events we send.
    lookahead = time_factory.makeInterval(0.1)
    rti_ambassador.enableTimeRegulation(lookahead)
    
    while not ambassador.is_time_regulating:
        rti_ambassador.evokeCallback(0.1)

    # 4. Synchronization Point
    # Register a sync point so all federates start at the same time.
    SYNC_POINT_NAME = "ReadyToStart"
    rti_ambassador.registerFederationSynchronizationPoint(SYNC_POINT_NAME, bytearray())
    
    print("Esperando a que el usuario presione Enter para Sincronizar...")
    input("Presiona Enter para confirmar que estamos listos...")
    
    # Achieve the sync point (signal that we are ready)
    rti_ambassador.synchronizationPointAchieved(SYNC_POINT_NAME)
    print(f"Achieved Sync Point: {SYNC_POINT_NAME}. Esperando a que todos se sincronicen...")
    
    while not ambassador.is_synchronized:
        rti_ambassador.evokeCallback(0.1)
        time.sleep(0.1)
        
    print("Todos sincronizados. Iniciando simulación...")

    # 5. Publish Object
    sine_handle = rti_ambassador.getObjectClassHandle("SineWave")
    val_handle = rti_ambassador.getAttributeHandle(sine_handle, "value")
    
    pub_attributes = rti_ambassador.getAttributeHandleSetFactory().create()
    pub_attributes.add(val_handle)
    rti_ambassador.publishObjectClassAttributes(sine_handle, pub_attributes)
    
    instance_handle = rti_ambassador.registerObjectInstance(sine_handle)

    # 6. Simulation Loop (Time Managed)
    duration = 10.0
    step = 0.25
    
    # Start at current logical time (usually 0.0)
    logical_time = ambassador.current_time
    
    while logical_time <= duration:
        # Calculate Sine
        sine_val = math.sin(logical_time)
        msg = f"{sine_val:.4f}"
        
        # Encode
        encoder = encoder_factory.createHLAunicodeString()
        encoder.setValue(msg)
        
        # Prepare attributes
        attribute_values = rti_ambassador.getAttributeHandleValueMapFactory().create(1)
        attribute_values.put(val_handle, encoder.toByteArray())
        
        # Send update (Receive Order as per FOM, but regulated by time advancement)
        rti_ambassador.updateAttributeValues(instance_handle, attribute_values, bytearray())
        
        print(f"Valor del seno en t={logical_time:.2f}: {msg}")
        
        # Request Time Advance
        next_time = logical_time + step
        hla_next_time = time_factory.makeTime(next_time)
        rti_ambassador.timeAdvanceRequest(hla_next_time)
        
        # Wait for Grant (TAG)
        while ambassador.current_time < next_time:
            rti_ambassador.evokeCallback(0.1)
        
        logical_time = ambassador.current_time
        
        # Visual sleep (optional, just to see it running)
        time.sleep(0.25)

    # Send END message
    encoder.setValue("END")
    attribute_values.put(val_handle, encoder.toByteArray())
    rti_ambassador.updateAttributeValues(instance_handle, attribute_values, bytearray())
    print("Enviado mensaje de fin.")

    # Cleanup
    rti_ambassador.resignFederationExecution(ResignAction.DELETE_OBJECTS)
    try:
        rti_ambassador.destroyFederationExecution("SineFed")
        print("Federation destroyed.")
    except Exception as e:
        print(f"Could not destroy federation (maybe others are still connected?): {e}")
    
    print("Ejecución finalizada.")

if __name__ == "__main__":
    run_publisher()
