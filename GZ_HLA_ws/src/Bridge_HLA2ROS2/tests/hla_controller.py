import jpype
import jpype.imports
import time
import json
import sys

# --- CONFIGURATION ---
PITCH_JAR = "/home/vicen/prti1516e/lib/prti1516e.jar"
if not jpype.isJVMStarted():
    jpype.startJVM(classpath=[PITCH_JAR])

from hla.rti1516e import RtiFactoryFactory, CallbackModel, ResignAction

@jpype.JImplements("hla.rti1516e.FederateAmbassador")
class ControllerAmbassador(object):
    def __init__(self, controller):
        self.controller = controller

    @jpype.JOverride
    def discoverObjectInstance(self, objectHandle, objectClassHandle, objectName):
        print(f"Discovered Object: {objectName}")
        self.controller.discovered_object = objectHandle

    @jpype.JOverride
    def reflectAttributeValues(self, objectHandle, attributeValues, tag, sentOrder, transport, reflectInfo=None):
        # Decode pose
        # We assume we know the handle for pose
        pose_handle = self.controller.pose_handle
        if attributeValues.containsKey(pose_handle):
            decoder = self.controller.encoder_factory.createHLAunicodeString()
            decoder.decode(attributeValues.get(pose_handle))
            val = decoder.getValue()
            print(f"Controller Received Pose: {val}")

    # Empty implementations
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
    def timeAdvanceGrant(self, time, *args): pass
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

class Controller:
    def __init__(self):
        self.rti_factory = RtiFactoryFactory.getRtiFactory()
        self.rti_ambassador = self.rti_factory.getRtiAmbassador()
        self.encoder_factory = self.rti_factory.getEncoderFactory()
        self.ambassador = ControllerAmbassador(self)
        self.discovered_object = None
        self.pose_handle = None
        self.cmd_vel_handle = None

    def run(self):
        try:
            self.rti_ambassador.connect(self.ambassador, CallbackModel.HLA_EVOKED)
        except Exception: pass
        
        try:
            self.rti_ambassador.joinFederationExecution("Controller", "ControllerType", "RosHlaFed")
        except Exception as e:
            print(f"Join failed (maybe fed doesn't exist yet?): {e}")
            return

        # Get Handles
        turtlesim_handle = self.rti_ambassador.getObjectClassHandle("Turtlesim")
        self.pose_handle = self.rti_ambassador.getAttributeHandle(turtlesim_handle, "pose")
        self.cmd_vel_handle = self.rti_ambassador.getAttributeHandle(turtlesim_handle, "cmd_vel")

        # Subscribe to Pose
        attr_set = self.rti_ambassador.getAttributeHandleSetFactory().create()
        attr_set.add(self.pose_handle)
        self.rti_ambassador.subscribeObjectClassAttributes(turtlesim_handle, attr_set)

        # Publish CmdVel
        attr_set_pub = self.rti_ambassador.getAttributeHandleSetFactory().create()
        attr_set_pub.add(self.cmd_vel_handle)
        self.rti_ambassador.publishObjectClassAttributes(turtlesim_handle, attr_set_pub)

        print("Controller waiting for object...")
        while self.discovered_object is None:
            self.rti_ambassador.evokeCallback(0.1)
            time.sleep(0.1)

        # Acquire Ownership of cmd_vel
        print("Acquiring ownership of cmd_vel...")
        try:
            attr_set_own = self.rti_ambassador.getAttributeHandleSetFactory().create()
            attr_set_own.add(self.cmd_vel_handle)
            self.rti_ambassador.attributeOwnershipAcquisition(self.discovered_object, attr_set_own, None)
        except Exception as e:
            print(f"Ownership acquisition failed: {e}")

        # Loop
        print("Starting Control Loop...")
        count = 0
        while True:
            self.rti_ambassador.evokeCallback(0.1)
            
            # Send cmd_vel every 2 seconds
            if count % 20 == 0:
                msg = {"linear": {"x": 2.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 1.0}}
                json_str = json.dumps(msg)
                
                attributes = self.rti_ambassador.getAttributeHandleValueMapFactory().create(1)
                encoder = self.encoder_factory.createHLAunicodeString(json_str)
                attributes.put(self.cmd_vel_handle, encoder.toByteArray())
                
                try:
                    self.rti_ambassador.updateAttributeValues(self.discovered_object, attributes, None)
                    print("Sent cmd_vel")
                except Exception as e:
                    print(f"Error sending cmd_vel: {e}")
            
            count += 1
            time.sleep(0.1)

if __name__ == "__main__":
    c = Controller()
    c.run()
