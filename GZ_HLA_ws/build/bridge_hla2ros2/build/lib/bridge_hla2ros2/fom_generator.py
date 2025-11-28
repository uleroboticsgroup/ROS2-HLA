import yaml
import xml.etree.ElementTree as ET
import xml.dom.minidom

def generate_fom(config_path, output_path):
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    root = ET.Element('objectModel', {
        'xsi:schemaLocation': 'http://standards.ieee.org/IEEE1516-2010/2010 http://standards.ieee.org/downloads/1516/1516.2-2010/IEEE1516-DIF-2010.xsd',
        'xmlns': 'http://standards.ieee.org/IEEE1516-2010',
        'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance'
    })

    # Model Identification
    model_id = ET.SubElement(root, 'modelIdentification')
    ET.SubElement(model_id, 'name').text = config.get('fom_name', 'UniversalRosFom')
    ET.SubElement(model_id, 'type').text = 'FOM'

    # Objects
    objects = ET.SubElement(root, 'objects')
    obj_root = ET.SubElement(objects, 'objectClass')
    ET.SubElement(obj_root, 'name').text = 'HLAobjectRoot'

    for node_name, node_data in config.get('nodes', {}).items():
        obj_class_name = node_data.get('object_class', node_name.capitalize())
        
        obj_class = ET.SubElement(obj_root, 'objectClass')
        ET.SubElement(obj_class, 'name').text = obj_class_name
        ET.SubElement(obj_class, 'sharing').text = 'PublishSubscribe'

        for attr_name, attr_data in node_data.get('attributes', {}).items():
            attr = ET.SubElement(obj_class, 'attribute')
            ET.SubElement(attr, 'name').text = attr_name
            ET.SubElement(attr, 'dataType').text = 'HLAunicodeString' # Universal type
            ET.SubElement(attr, 'updateType').text = 'Static'
            ET.SubElement(attr, 'transportation').text = 'HLAreliable'
            ET.SubElement(attr, 'order').text = 'Receive'

    # Interactions (Empty for now)
    interactions = ET.SubElement(root, 'interactions')
    int_root = ET.SubElement(interactions, 'interactionClass')
    ET.SubElement(int_root, 'name').text = 'HLAinteractionRoot'

    # Dimensions, Transportations, Switches, UpdateRates (Standard boilerplate)
    ET.SubElement(root, 'dimensions')
    ET.SubElement(root, 'transportations')
    
    switches = ET.SubElement(root, 'switches')
    ET.SubElement(switches, 'autoProvide', {'isEnabled': 'false'})
    ET.SubElement(switches, 'conveyRegionDesignatorSets', {'isEnabled': 'false'})
    ET.SubElement(switches, 'conveyProducingFederate', {'isEnabled': 'false'})
    ET.SubElement(switches, 'attributeScopeAdvisory', {'isEnabled': 'false'})
    ET.SubElement(switches, 'attributeRelevanceAdvisory', {'isEnabled': 'false'})
    ET.SubElement(switches, 'objectClassRelevanceAdvisory', {'isEnabled': 'false'})
    ET.SubElement(switches, 'interactionRelevanceAdvisory', {'isEnabled': 'false'})
    ET.SubElement(switches, 'serviceReporting', {'isEnabled': 'false'})
    ET.SubElement(switches, 'exceptionReporting', {'isEnabled': 'false'})
    ET.SubElement(switches, 'delaySubscriptionEvaluation', {'isEnabled': 'false'})
    ET.SubElement(switches, 'automaticResignAction', {'resignAction': 'CancelThenDeleteThenDivest'})

    ET.SubElement(root, 'updateRates')

    # Pretty print
    xml_str = ET.tostring(root, encoding='utf-8')
    parsed = xml.dom.minidom.parseString(xml_str)
    pretty_xml = parsed.toprettyxml(indent="    ")

    with open(output_path, 'w') as f:
        f.write(pretty_xml)
    
    print(f"FOM generated at {output_path}")

if __name__ == "__main__":
    generate_fom('bridge_config.yaml', 'universal_fom.xml')
