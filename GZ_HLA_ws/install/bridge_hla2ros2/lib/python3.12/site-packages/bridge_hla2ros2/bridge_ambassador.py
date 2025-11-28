
import jpype
import jpype.imports

# --- Ambassador ---
@jpype.JImplements("hla.rti1516e.FederateAmbassador")
class BridgeAmbassador(object):
    def __init__(self, bridge):
        self.bridge = bridge

    @jpype.JOverride
    def reflectAttributeValues(self, objectHandle, attributeValues, tag, sentOrder, transport, *args):
        try:
            # args can contain (time, receivedOrder, reflectInfo) or just (reflectInfo) depending on callback
            # We just process the attributes
            for handle in attributeValues.keySet():
                self.bridge.hla_attribute_updated(objectHandle, handle, attributeValues.get(handle))
        except Exception as e:
            print(f"Error in reflectAttributeValues: {e}")
            import traceback
            traceback.print_exc()

    @jpype.JOverride
    def discoverObjectInstance(self, objectHandle, objectClassHandle, objectName):
        try:
            self.bridge.discoverObjectInstance(objectHandle, objectClassHandle, objectName)
        except Exception as e:
            print(f"Error in discoverObjectInstance: {e}")
            import traceback
            traceback.print_exc()

    @jpype.JOverride
    def timeRegulationEnabled(self, time, *args):
        try:
            self.bridge.timeRegulationEnabled(time, *args)
        except Exception as e:
            print(f"Error in timeRegulationEnabled: {e}")
            import traceback
            traceback.print_exc()

    @jpype.JOverride
    def timeConstrainedEnabled(self, time, *args):
        try:
            self.bridge.timeConstrainedEnabled(time, *args)
        except Exception as e:
            print(f"Error in timeConstrainedEnabled: {e}")
            import traceback
            traceback.print_exc()

    @jpype.JOverride
    def timeAdvanceGrant(self, time, *args):
        try:
            self.bridge.timeAdvanceGrant(time, *args)
        except Exception as e:
            print(f"Error in timeAdvanceGrant: {e}")
            import traceback
            traceback.print_exc()

    @jpype.JOverride
    def objectInstanceNameReservationSucceeded(self, objectName, *args):
        print(f"Name reserved: {objectName}")
        self.bridge.reserved_names.add(objectName)

    @jpype.JOverride
    def objectInstanceNameReservationFailed(self, objectName, *args):
        print(f"Name reservation failed: {objectName}")
        self.bridge.failed_reservations.add(objectName)

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
    def requestAttributeOwnershipRelease(self, instance_handle, attr_handles, tag):
        try:
            self.bridge.requestAttributeOwnershipRelease(instance_handle, attr_handles, tag)
        except Exception as e:
            print(f"Error in requestAttributeOwnershipRelease: {e}")
            import traceback
            traceback.print_exc()

    @jpype.JOverride
    def attributeOwnershipAcquisitionNotification(self, instance_handle, attr_handles, tag):
        try:
            self.bridge.attributeOwnershipAcquisitionNotification(instance_handle, attr_handles, tag)
        except Exception as e:
            print(f"Error in attributeOwnershipAcquisitionNotification: {e}")
            import traceback
            traceback.print_exc()

    @jpype.JOverride
    def attributeOwnershipUnavailable(self, instance_handle, attr_handles):
        try:
            self.bridge.attributeOwnershipUnavailable(instance_handle, attr_handles)
        except Exception as e:
            print(f"Error in attributeOwnershipUnavailable: {e}")
            import traceback
            traceback.print_exc()
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
