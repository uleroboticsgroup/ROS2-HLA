import jpype
import jpype.imports
import os

def main():
    jar_path = "/home/vicen/prti1516e/lib/prti1516e.jar"
    jar_dir = os.path.dirname(jar_path)
    core_jar = os.path.join(jar_dir, 'prticore.jar')
    classpath = [jar_path]
    if os.path.exists(core_jar):
        classpath.append(core_jar)
        
    if not jpype.isJVMStarted():
        jpype.startJVM(classpath=classpath)
    
    from hla.rti1516e import LogicalTimeFactoryFactory
    
    try:
        time_factory = LogicalTimeFactoryFactory.getLogicalTimeFactory("HLAfloat64Time")
        print(f"Got factory: {time_factory}")
        
        interval = time_factory.makeInterval()
        print(f"Created interval: {interval}")
        
        time_obj = time_factory.makeTime()
        print(f"Created time: {time_obj}")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
