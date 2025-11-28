import jpype
import jpype.imports
import time

# --- CONFIGURACIÓN (Misma que Chatter) ---
PITCH_JAR = "/home/vicen/prti1516e/lib/prti1516e.jar" 
jpype.startJVM(classpath=[PITCH_JAR])

from hla.rti1516e import RtiFactoryFactory, NullFederateAmbassador, CallbackModel

# --- LA CLASE QUE ESCUCHA (El Listener) ---
# Implementamos la interfaz Java en Python
@jpype.JImplements("hla.rti1516e.FederateAmbassador")
class MyListener(object):
    def __init__(self, encoder_factory):
        self.encoder_factory = encoder_factory

    @jpype.JOverride
    @jpype.JOverride
    def reflectAttributeValues(self, objectHandle, attributeValues, tag, sentOrder, transport, reflectInfo=None):
        # Note: reflectInfo might be passed as the 6th argument if using a different mapping, 
        # but for standard 1516e without time, it's often just these args or with an extra one.
        # We'll use the signature that matches what the RTI expects for 'receive' order.
        # Actually, the standard signature for reflectAttributeValues without time is:
        # (ObjectInstanceHandle, AttributeHandleValueMap, byte[], OrderType, TransportationTypeLabel, SupplementalReflectInfo)
        
        for handle in attributeValues.keySet():
            decoder = self.encoder_factory.createHLAunicodeString()
            decoder.decode(attributeValues.get(handle))
            print(f"I heard: '{decoder.getValue()}'")

    @jpype.JOverride
    def connectionLost(self, *args): pass
    @jpype.JOverride
    def reportFederationExecutions(self, *args): pass
    @jpype.JOverride
    def synchronizationPointRegistrationSucceeded(self, *args): pass
    @jpype.JOverride
    def synchronizationPointRegistrationFailed(self, *args): pass
    @jpype.JOverride
    def federationSynchronized(self, *args): pass
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
    def discoverObjectInstance(self, *args): pass
    @jpype.JOverride
    def receiveInteraction(self, *args): pass
    @jpype.JOverride
    def removeObjectInstance(self, *args): pass
    @jpype.JOverride
    def attributesInScope(self, *args): pass
    @jpype.JOverride
    def attributesOutOfScope(self, *args): pass
    @jpype.JOverride
    def provideAttributeValueUpdate(self, *args): pass
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
    def timeConstrainedEnabled(self, *args): pass
    @jpype.JOverride
    def timeAdvanceGrant(self, *args): pass
    @jpype.JOverride
    def requestRetraction(self, *args): pass
    @jpype.JOverride
    def announceSynchronizationPoint(self, *args): pass

def run_listener():
    rti_factory = RtiFactoryFactory.getRtiFactory()
    rti_ambassador = rti_factory.getRtiAmbassador()
    encoder_factory = rti_factory.getEncoderFactory()
    
    # Instanciamos nuestro Listener personalizado
    my_listener = MyListener(encoder_factory)
    
    # Conectar y Unir (Igual que Chatter pero con otro nombre)
    rti_ambassador.connect(my_listener, CallbackModel.HLA_EVOKED) # Pasamos nuestro listener
    
    connected = False
    while not connected:
        try:
            rti_ambassador.joinFederationExecution("Listener01", "ListenerType", "ChatterFed")
            connected = True
            print("Joined federation successfully.")
        except Exception as e:
            print(f"Waiting for Chatter to create federation... ({e})")
            time.sleep(2)

    # SUSCRIBIRSE (Decir "Quiero escuchar esto")
    comm_handle = rti_ambassador.getObjectClassHandle("Communication")
    msg_handle = rti_ambassador.getAttributeHandle(comm_handle, "Message")
    
    attributes = rti_ambassador.getAttributeHandleSetFactory().create()
    attributes.add(msg_handle)
    
    rti_ambassador.subscribeObjectClassAttributes(comm_handle, attributes)
    print("Escuchando topic 'Communication'...")

    # Bucle de escucha
    while True:
        # Esto es CRÍTICO. Sin evokeCallback, no llegan mensajes.
        # Es como el ros::spin()
        rti_ambassador.evokeCallback(1.0) 

if __name__ == "__main__":
    run_listener()