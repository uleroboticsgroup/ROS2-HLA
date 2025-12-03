import xml.etree.ElementTree as ET
import xml.dom.minidom

class FOMGenerator:
    def __init__(self):
        self.objects = {} # class_name -> { attributes: {name: type} }
        self.interactions = {} # class_name -> { parameters: {name: type} }

    def generate(self, config, output_path):
        self._parse_config(config.get('ros_to_hla', []))
        self._parse_config(config.get('ros_to_hla', []))
        self._parse_config(config.get('hla_to_ros', []))
        self._parse_actions(config.get('ros_actions', []))
        
        xml_content = self._build_xml()
        
        with open(output_path, 'w') as f:
            f.write(xml_content)
            
    def _parse_config(self, config_list):
        for item in config_list:
            if 'hla_object_class' in item:
                cls = item['hla_object_class']
                if cls not in self.objects:
                    self.objects[cls] = {'attributes': {}}
                
                # Attributes from mapping
                for _, attr in item.get('mapping', {}).items():
                    self.objects[cls]['attributes'][attr] = 'HLAfloat64BE' # Default
                    
                # Attributes from fixed_parameters (if any, though usually for interactions)
                for param in item.get('fixed_parameters', {}).keys():
                     self.objects[cls]['attributes'][param] = 'HLAunicodeString'

            elif 'hla_interaction_class' in item:
                cls = item['hla_interaction_class']
                if cls not in self.interactions:
                    self.interactions[cls] = {'parameters': {}}
                
                # Parameters from mapping
                for _, param in item.get('mapping', {}).items():
                    self.interactions[cls]['parameters'][param] = 'HLAfloat64BE' # Default
                
                # Parameters from fixed_parameters
                for param in item.get('fixed_parameters', {}).keys():
                    self.interactions[cls]['parameters'][param] = 'HLAunicodeString'
                
                # Filter parameter
                if 'filter_parameter' in item:
                    self.interactions[cls]['parameters'][item['filter_parameter']] = 'HLAunicodeString'

    def _parse_actions(self, actions_list):
        for item in actions_list:
            # Goal Interaction
            goal_cls = item['hla_goal_interaction']
            if goal_cls not in self.interactions:
                self.interactions[goal_cls] = {'parameters': {}}
            
            for _, param in item.get('mapping', {}).get('goal', {}).items():
                self.interactions[goal_cls]['parameters'][param] = 'HLAfloat64BE' # Default
                
            # Result Interaction
            res_cls = item['hla_result_interaction']
            if res_cls not in self.interactions:
                self.interactions[res_cls] = {'parameters': {}}
                
            for _, param in item.get('mapping', {}).get('result', {}).items():
                self.interactions[res_cls]['parameters'][param] = 'HLAfloat64BE' # Default

    def _build_xml(self):
        root = ET.Element('objectModel', {
            'xsi:schemaLocation': "http://standards.ieee.org/IEEE1516-2010/2010 http://standards.ieee.org/downloads/1516/1516.2-2010/IEEE1516-DIF-2010.xsd",
            'xmlns': "http://standards.ieee.org/IEEE1516-2010",
            'xmlns:xsi': "http://www.w3.org/2001/XMLSchema-instance"
        })
        
        # Model ID
        model_id = ET.SubElement(root, 'modelIdentification')
        ET.SubElement(model_id, 'name').text = 'GeneratedRoboticsFOM'
        ET.SubElement(model_id, 'type').text = 'FOM'
        
        # Objects
        objects_elem = ET.SubElement(root, 'objects')
        obj_root = ET.SubElement(objects_elem, 'objectClass')
        ET.SubElement(obj_root, 'name').text = 'HLAobjectRoot'
        
        for cls_name, data in self.objects.items():
            cls_elem = ET.SubElement(obj_root, 'objectClass')
            ET.SubElement(cls_elem, 'name').text = cls_name
            ET.SubElement(cls_elem, 'sharing').text = 'PublishSubscribe'
            
            for attr_name, attr_type in data['attributes'].items():
                attr_elem = ET.SubElement(cls_elem, 'attribute')
                ET.SubElement(attr_elem, 'name').text = attr_name
                ET.SubElement(attr_elem, 'dataType').text = attr_type
                ET.SubElement(attr_elem, 'updateType').text = 'Periodic'
                ET.SubElement(attr_elem, 'transportation').text = 'HLAreliable'
                ET.SubElement(attr_elem, 'order').text = 'Receive'

        # Interactions
        interactions_elem = ET.SubElement(root, 'interactions')
        int_root = ET.SubElement(interactions_elem, 'interactionClass')
        ET.SubElement(int_root, 'name').text = 'HLAinteractionRoot'
        
        for cls_name, data in self.interactions.items():
            cls_elem = ET.SubElement(int_root, 'interactionClass')
            ET.SubElement(cls_elem, 'name').text = cls_name
            ET.SubElement(cls_elem, 'sharing').text = 'PublishSubscribe'
            ET.SubElement(cls_elem, 'transportation').text = 'HLAreliable'
            ET.SubElement(cls_elem, 'order').text = 'Receive'
            
            for param_name, param_type in data['parameters'].items():
                param_elem = ET.SubElement(cls_elem, 'parameter')
                ET.SubElement(param_elem, 'name').text = param_name
                ET.SubElement(param_elem, 'dataType').text = param_type

        # Boilerplate
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
        return parsed.toprettyxml(indent="    ")
