import jpype
import jpype.imports
import time
import json
import math

# Start JVM
if not jpype.isJVMStarted():
    jpype.startJVM(classpath=["/home/vicen/prti1516e/lib/prti1516e.jar"])

from hla.rti1516e import RtiFactoryFactory, NullFederateAmbassador

@jpype.JImplements("hla.rti1516e.FederateAmbassador")
class TurtleBotController(object):
    def __init__(self):
        self.rti_factory = RtiFactoryFactory.getRtiFactory()
        self.rti_ambassador = self.rti_factory.getRtiAmbassador()
        self.encoder_factory = self.rti_factory.getEncoderFactory()
        self.object_handle = None
        self.cmd_vel_handle = None
        self.undock_handle = None
        self.running = True

    def run(self):
        # Import CallbackModel
        from hla.rti1516e import CallbackModel
        try:
            self.rti_ambassador.connect(self, CallbackModel.HLA_EVOKED)
        except Exception as e:
            print(f"Connect warning: {e}")
        
        # We assume Federation is already created by the Bridge
        # But if we wanted to create it, we need a URL. 
        # Since Bridge runs first, we can skip creation or handle it properly.
        # For now, let's just skip creation and assume Bridge did it.
        # If we really need to create, we need the FOM path.
        
        # Retry joining
        joined = False
        for i in range(10):
            try:
                self.rti_ambassador.joinFederationExecution("Controller", "ControllerType", "TurtleBotFed2")
                joined = True
                print("Joined Federation!")
                break
            except Exception as e:
                 print(f"Join failed (attempt {i+1}): {e}")
                 time.sleep(1.0)
        
        if not joined:
            return
        
        # Subscribe to TurtleBot
        object_class = self.rti_ambassador.getObjectClassHandle("TurtleBot")
        self.cmd_vel_handle = self.rti_ambassador.getAttributeHandle(object_class, "cmd_vel")
        self.undock_handle = self.rti_ambassador.getAttributeHandle(object_class, "undock_trigger")
        pose_handle = self.rti_ambassador.getAttributeHandle(object_class, "pose")
        
        # Publish cmd_vel and undock_trigger
        pub_attributes = self.rti_ambassador.getAttributeHandleSetFactory().create()
        pub_attributes.add(self.cmd_vel_handle)
        pub_attributes.add(self.undock_handle)
        self.rti_ambassador.publishObjectClassAttributes(object_class, pub_attributes)
        
        # Subscribe to pose (to discover object)
        sub_attributes = self.rti_ambassador.getAttributeHandleSetFactory().create()
        sub_attributes.add(pose_handle)
        self.rti_ambassador.subscribeObjectClassAttributes(object_class, sub_attributes)
        
        print("Waiting for TurtleBot discovery...")
        while self.object_handle is None:
            self.rti_ambassador.evokeCallback(0.1)
            time.sleep(0.1)
            
        # Acquire Ownership
        print("Acquiring ownership...")
        try:
            attr_set_own = self.rti_ambassador.getAttributeHandleSetFactory().create()
            attr_set_own.add(self.cmd_vel_handle)
            attr_set_own.add(self.undock_handle)
            self.rti_ambassador.attributeOwnershipAcquisition(self.object_handle, attr_set_own, None)
            # Wait for ownership? Usually instant if unowned.
            # But we should probably wait a bit or handle callback.
            # For simplicity, just wait a second.
            time.sleep(1.0)
        except Exception as e:
            print(f"Ownership acquisition failed: {e}")
        
        # 1. Trigger Undock
        print("Triggering Undock...")
        self.send_undock()
        
        print("Waiting 20 seconds for undock...")
        for i in range(20):
            self.rti_ambassador.evokeCallback(1.0)
            time.sleep(1.0)
            print(f"Wait: {i+1}/20")
            
        # 2. Drive in Circles
        print("Driving in circles...")
        t = 0
        try:
            while True:
                self.send_circle_command()
                self.rti_ambassador.evokeCallback(0.1)
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
            
        self.rti_ambassador.resignFederationExecution(self.rti_ambassador.ResignAction.DELETE_OBJECTS)

    def discoverObjectInstance(self, objectHandle, objectClassHandle, objectName):
        print(f"Discovered {objectName}")
        self.object_handle = objectHandle

    def send_undock(self):
        if self.object_handle is None: return
        
        # JSON for std_msgs/Bool
        json_str = json.dumps({"data": True})
        
        attributes = self.rti_ambassador.getAttributeHandleValueMapFactory().create(1)
        encoder = self.encoder_factory.createHLAunicodeString()
        encoder.setValue(json_str)
        attributes.put(self.undock_handle, encoder.toByteArray())
        
        self.rti_ambassador.updateAttributeValues(self.object_handle, attributes, None)
        print("Undock command sent.")

    def send_circle_command(self):
        if self.object_handle is None: return
        
        # Twist JSON
        cmd = {
            "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.5}
        }
        json_str = json.dumps(cmd)
        
        attributes = self.rti_ambassador.getAttributeHandleValueMapFactory().create(1)
        encoder = self.encoder_factory.createHLAunicodeString()
        encoder.setValue(json_str)
        attributes.put(self.cmd_vel_handle, encoder.toByteArray())
        
        self.rti_ambassador.updateAttributeValues(self.object_handle, attributes, None)

    # Required Ambassador Methods
    @jpype.JOverride
    def synchronizationPointRegistrationSucceeded(self, label, *args): pass
    @jpype.JOverride
    def synchronizationPointRegistrationFailed(self, label, reason, *args): pass
    @jpype.JOverride
    def announceSynchronizationPoint(self, label, tag, *args): pass
    @jpype.JOverride
    def federationSynchronized(self, label, *args): pass
    @jpype.JOverride
    def timeConstrainedEnabled(self, time, *args): pass
    @jpype.JOverride
    def timeRegulationEnabled(self, time, *args): pass
    @jpype.JOverride
    def timeAdvanceGrant(self, time, *args): pass
    @jpype.JOverride
    def discoverObjectInstance(self, objectHandle, objectClassHandle, objectName):
        print(f"Discovered {objectName}")
        self.object_handle = objectHandle
    @jpype.JOverride
    def reflectAttributeValues(self, *args): pass
    @jpype.JOverride
    def provideAttributeValueUpdate(self, *args): pass
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

if __name__ == "__main__":
    ctrl = TurtleBotController()
    ctrl.run()
