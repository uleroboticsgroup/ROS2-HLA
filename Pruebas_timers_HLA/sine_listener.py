import jpype
import jpype.imports
import time
import sys

# --- CONFIGURATION ---
PITCH_JAR = "/home/vicen/prti1516e/lib/prti1516e.jar"
jpype.startJVM(classpath=[PITCH_JAR])

from hla.rti1516e import RtiFactoryFactory, CallbackModel, ResignAction

# --- LISTENER CLASS ---
@jpype.JImplements("hla.rti1516e.FederateAmbassador")
class ListenerAmbassador(object):
    def __init__(self, encoder_factory):
        self.encoder_factory = encoder_factory
        self.is_synchronized = False
        self.is_time_constrained = False
        self.current_time = 0.0
        self.running = True

    @jpype.JOverride
    def announceSynchronizationPoint(self, label, tag, *args):
        print(f"Sync Point Anunciado: {label}")

    @jpype.JOverride
    def federationSynchronized(self, label, *args):
        print(f"Federación Sincronizada: {label}")
        self.is_synchronized = True

    @jpype.JOverride
    def timeConstrainedEnabled(self, time, *args):
        self.current_time = time.getValue()
        self.is_time_constrained = True
        print(f"Time Constrained Enabled. Current Time: {self.current_time}")

    @jpype.JOverride
    def timeAdvanceGrant(self, time, *args):
        self.current_time = time.getValue()

    @jpype.JOverride
    def reflectAttributeValues(self, objectHandle, attributeValues, tag, sentOrder, transport, reflectInfo=None):
        for handle in attributeValues.keySet():
            decoder = self.encoder_factory.createHLAunicodeString()
            decoder.decode(attributeValues.get(handle))
            val = decoder.getValue()
            
            if val == "END":
                print("Recibido mensaje de fin. Terminando...")
                self.running = False
            else:
                print(f"Recibido en T={self.current_time:.2f}: {val}")

    # --- Empty implementations for unused callbacks ---
    @jpype.JOverride
    def synchronizationPointRegistrationSucceeded(self, label, *args): pass
    @jpype.JOverride
    def synchronizationPointRegistrationFailed(self, label, reason, *args): pass
    @jpype.JOverride
    def discoverObjectInstance(self, *args): pass
    @jpype.JOverride
    def provideAttributeValueUpdate(self, *args): pass
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
    def timeRegulationEnabled(self, *args): pass
    @jpype.JOverride
    def requestRetraction(self, *args): pass


def run_listener():
    rti_factory = RtiFactoryFactory.getRtiFactory()
    rti_ambassador = rti_factory.getRtiAmbassador()
    encoder_factory = rti_factory.getEncoderFactory()
    
    ambassador = ListenerAmbassador(encoder_factory)
    
    # 1. Connect
    try:
        rti_ambassador.connect(ambassador, CallbackModel.HLA_EVOKED)
    except Exception as e:
        print(f"Error connecting: {e}")
        return

    # 2. Join Federation
    connected = False
    while not connected:
        try:
            rti_ambassador.joinFederationExecution("SineListener", "SineListenerType", "SineFed")
            connected = True
            print("Joined federation as SineListener.")
        except Exception as e:
            print(f"Waiting for federation... ({e})")
            time.sleep(2)

    # Get Time Factory AFTER joining
    time_factory = rti_ambassador.getTimeFactory()

    # 3. Enable Time Constrained
    # We want to receive messages in order and constrained by the publisher's time.
    rti_ambassador.enableTimeConstrained()
    
    while not ambassador.is_time_constrained:
        rti_ambassador.evokeCallback(0.1)

    # 4. Subscribe
    sine_handle = rti_ambassador.getObjectClassHandle("SineWave")
    val_handle = rti_ambassador.getAttributeHandle(sine_handle, "value")
    
    sub_attributes = rti_ambassador.getAttributeHandleSetFactory().create()
    sub_attributes.add(val_handle)
    rti_ambassador.subscribeObjectClassAttributes(sine_handle, sub_attributes)

    # 5. Synchronization Point
    print("Esperando Sync Point 'ReadyToStart'...")
    sync_point_name = "ReadyToStart"
    has_achieved = False
    
    while ambassador.running:
        # Try to achieve sync point if not done yet
        if not has_achieved:
             try:
                 rti_ambassador.synchronizationPointAchieved(sync_point_name)
                 print(f"Achieved Sync Point: {sync_point_name}")
                 has_achieved = True
             except Exception:
                 # Not announced yet by publisher, keep waiting
                 pass
        
        # If synchronized, proceed with time advancement
        if ambassador.is_synchronized:
             # Request next time step
             step = 0.25
             next_time = ambassador.current_time + step
             hla_next_time = time_factory.makeTime(next_time)
             
             rti_ambassador.timeAdvanceRequest(hla_next_time)
             
             # Wait for Grant (TAG)
             while ambassador.current_time < next_time and ambassador.running:
                 rti_ambassador.evokeCallback(0.1)
        else:
             # Just pump callbacks waiting for sync
             rti_ambassador.evokeCallback(0.1)
             time.sleep(0.1)

    # Cleanup
    rti_ambassador.resignFederationExecution(ResignAction.DELETE_OBJECTS)
    print("Ejecución finalizada.")

if __name__ == "__main__":
    run_listener()
