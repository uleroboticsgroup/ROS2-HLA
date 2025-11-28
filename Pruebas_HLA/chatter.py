import jpype
import jpype.imports
import time

# --- CONFIGURACIÓN DEL PUENTE JAVA ---
# Ajuste esta ruta a donde instaló Pitch (el archivo .jar es clave)
PITCH_JAR = "/home/vicen/prti1516e/lib/prti1516e.jar" 
jpype.startJVM(classpath=[PITCH_JAR])

# Importar las clases del estándar HLA 1516e directamente desde el JAR
from hla.rti1516e import RtiFactoryFactory, NullFederateAmbassador, ResignAction, CallbackModel

# --- MI FEDERADO (EL CODIGO) ---
def run_chatter():
    # 1. Conectar a la RTI
    rti_factory = RtiFactoryFactory.getRtiFactory()
    rti_ambassador = rti_factory.getRtiAmbassador()
    encoder_factory = rti_factory.getEncoderFactory()
    
    # El FederateAmbassador maneja los callbacks (lo que recibimos)
    # Como es chatter, usamos uno nulo (no nos importa escuchar)
    fed_ambassador = NullFederateAmbassador()
    
    # 2. Gestión de la Federación (Crear/Unir)
    try:
        rti_ambassador.connect(fed_ambassador, CallbackModel.HLA_EVOKED)
        try:
            # Intentamos crear. Si ya existe, fallará silenciosamente (es normal)
            fom_url = jpype.java.io.File("/home/vicen/ISDEFE/Pruebas_HLA/chatter.xml").toURI().toURL()
            rti_ambassador.createFederationExecution("ChatterFed", [fom_url])
        except Exception as e:
            print(f"La federación ya existe o error al crear: {e}")

        rti_ambassador.joinFederationExecution("Chatter01", "ChatterType", "ChatterFed")
    except Exception as e:
        print(f"Error de conexión: {e}")
        return

    # 3. Publicar (Anunciar que vamos a hablar)
    # Obtenemos los "Handles" (IDs numéricos) de las clases
    comm_handle = rti_ambassador.getObjectClassHandle("Communication")
    msg_handle = rti_ambassador.getAttributeHandle(comm_handle, "Message")
    
    # Creamos un set de atributos (Set<AttributeHandle>)
    attributes = rti_ambassador.getAttributeHandleSetFactory().create()
    attributes.add(msg_handle)
    
    rti_ambassador.publishObjectClassAttributes(comm_handle, attributes)
    
    # 4. Registrar Instancia (Crear el objeto concreto)
    instance_handle = rti_ambassador.registerObjectInstance(comm_handle)
    print(f"Chatter iniciado. Objeto registrado: {instance_handle}")

    # 5. Bucle Principal (El 'while ros::ok()')
    count = 0
    while True:
        # Preparar el dato (Codificar string a bytes HLA)
        text = f"Hello World: {count}"
        encoder = encoder_factory.createHLAunicodeString()
        encoder.setValue(text)
        
        # Empaquetar en un mapa de atributos
        attribute_values = rti_ambassador.getAttributeHandleValueMapFactory().create(1)
        attribute_values.put(msg_handle, encoder.toByteArray())
        
        # ENVIAR A LA RED (El equivalente a publish())
        rti_ambassador.updateAttributeValues(instance_handle, attribute_values, bytearray())
        
        print(f"Publishing: '{text}'")
        count += 1
        
        # IMPORTANTE: Dar tiempo a la RTI para procesar
        rti_ambassador.evokeCallback(0.1)
        time.sleep(1) # Publicar a 1 Hz

if __name__ == "__main__":
    run_chatter()