import jpype
import jpype.imports

PITCH_JAR = "/home/vicen/prti1516e/lib/prti1516e.jar" 
jpype.startJVM(classpath=[PITCH_JAR])

from hla.rti1516e import RtiFactoryFactory

rti_factory = RtiFactoryFactory.getRtiFactory()
print("RtiFactory methods:")
for method in dir(rti_factory):
    if not method.startswith('_'):
        print(f" - {method}")

rti_ambassador = rti_factory.getRtiAmbassador()
print("\nRtiAmbassador methods (starting with 'g'):")
for method in dir(rti_ambassador):
    if method.startswith('g'):
        print(f" - {method}")

# Also check for AttributeHandleValueMapFactory
try:
    from hla.rti1516e import AttributeHandleValueMapFactory
    print("Found hla.rti1516e.AttributeHandleValueMapFactory")
except ImportError:
    print("hla.rti1516e.AttributeHandleValueMapFactory NOT found")

# Check if we can use java.util.HashSet
try:
    HashSet = jpype.JClass("java.util.HashSet")
    s = HashSet()
    print("Successfully created java.util.HashSet")
except Exception as e:
    print(f"Failed to create HashSet: {e}")


