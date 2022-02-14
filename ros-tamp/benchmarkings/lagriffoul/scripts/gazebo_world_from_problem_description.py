import xml.etree.ElementTree as ET
import os
import shutil
from classes.problem_to_gazebo_translator import ProblemToGazeboTranslator

           

if __name__ == '__main__':
    #print(os.listdir())     
    gazebo_translator=ProblemToGazeboTranslator()
    gazebo_output_file_path="benchmarkings/lagriffoul/problems/pb_3_sorting_objects/gazebo_worlds/pb_3_gazebo_world.xml"
    gazebo_translator.initializeGazeboScenario(gazebo_output_file_path)    
    indent_level=2 #acccording to the initial gazebo file    
    # gazebo_translator.initModel(indent_level)
    # gazebo_translator.addBreakLine()
    path="benchmarkings/lagriffoul/problems/pb_3_sorting_objects/problem_definitions/pb_3.xml"
    mytree = ET.parse(path)
    myroot = mytree.getroot()
    xml_objects=myroot.find("objects")
   
    for obj in xml_objects:
        # gazebo_translator.processObject(obj,indent_level+1)
        gazebo_translator.processObject(obj,indent_level)


    gazebo_translator.addBreakLine()

    # gazebo_translator.closeModel(indent_level)
    gazebo_translator.endGazeboScenario(gazebo_output_file_path)