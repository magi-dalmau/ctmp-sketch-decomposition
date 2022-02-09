import xml.etree.ElementTree as ET
import os
import shutil
from matplotlib import colors
from tf.transformations import euler_from_matrix
import numpy as np
import trimesh


class ProblemToGazeboTranslator:
    def __init__(self):
        self.gazebo_output = "\n"
        self.meshes_inertial_params={}
        self.position_to_letter=["x","y","z"]
        self.friction_coef=0.5

    def initModel(self,name, indent_level):
        self.addOpenTag("model", indent_level, params=[
                        {"name": name}])
        self.addBreakLine()
        if "table" in name:
            self.addOpenTag("static",indent_level+1)
            self.gazebo_output+="1"
            self.addCloseTag("static")
            self.addBreakLine()
            
        # self.addOpenTag("self_collide",indent_level+1)
        # self.gazebo_output+="1"
        # self.addCloseTag("self_collide")



    def closeModel(self, indent_level):
        self.addCloseTag("model", indent_level=indent_level)

    def processObject(self, obj, indent_level):
        #print("Processing object: "+obj.find("name").text)
        self.generateGazeboObject(obj, indent_level)
        # obj_info={
        # }
        # obj_info["name"]=obj.find("name").text
        # problem_objects.append(obj_info)

    def getColorFromName(self, name):
        for part in name.split("_"):
            try:
                color = colors.to_rgba(part)
                return color
            except:
                continue
        return (-1, -1, -1, -1)  # no color found code

    def parseTransformationMatrix(self, tf_matrix):
        # tf_matrix is encoded by rot_0_0,rot_0_1,rot_0_2,pos_1,rot_1_0,....,pos_3 note that last row 0 0 0 1 is not included
        pose = ""
        tf_matrix_doubles=[]       
        for t in tf_matrix.split():
            tf_matrix_doubles.append(float(t))
        #print(tf_matrix_doubles)
        pose += self.getPositionFromTFMatrix(tf_matrix_doubles) + " "
        pose += self.getRPYFromTFMatrix(tf_matrix_doubles)
        #print(pose)
        return pose
    def getPositionFromTFMatrix(self, tf_matrix):
        pos_tup = (tf_matrix[3], tf_matrix[7], tf_matrix[11])
        pos_string=""
        for p in pos_tup:
            pos_string+=str(p)+" "
        return pos_string[:-1]
    def getRPYFromTFMatrix(self, tf_matrix):
        euler_angles = euler_from_matrix(
            self.getRotationMatrixFromTFMatrix(tf_matrix))
        rpy_string = ""
        for angle in euler_angles:
            rpy_string += str(angle)+" "
        return rpy_string[:-1]

    def getRotationMatrixFromTFMatrix(self, tf_matrix):

        return np.array([[tf_matrix[0], tf_matrix[1], tf_matrix[2]], \
             [tf_matrix[4], tf_matrix[5], tf_matrix[6]], \
                  [tf_matrix[8], tf_matrix[9], tf_matrix[10]]])

    def generateGazeboObject(self, obj, indent_level):
        name = obj.find("name").text            
        pose = self.parseTransformationMatrix(obj.find("pose").text)
        meshes_path = "/benchmarkings/lagriffoul/problems/pb_3_sorting_objects/meshes/"
        local_uri = (obj.find("geom").text).split(".")[0]
        #uri = meshes_path+local_uri+".dae"
        uri = meshes_path+local_uri+".stl"        
        #print("generating gazebo object with name: "+name+" pose: "+pose+" and uri: "+uri)
        color = self.getColorFromName(local_uri)
        self.initModel(name,indent_level)
        self.addBreakLine()
        self.addOpenTag("link", indent_level+1, params=[{"name": name+"_link"}])
        self.addBreakLine()
        self.addPose(pose, indent_level+2)
        self.addBreakLine()
        self.addInertial(uri,local_uri,indent_level+2)
        self.addBreakLine()
        self.addVisualWithMesh( name+"_link", pose, uri, color, indent_level+2)
        self.addBreakLine()
        self.addCollisionWithMesh( name+"_link", pose, uri, indent_level+2)
        self.addBreakLine()
        self.addCloseTag("link", indent_level=indent_level+1)
        self.addBreakLine()
        self.closeModel(indent_level)
        self.addBreakLine()
        #print ("Output now is: \n"+self.gazebo_output)

    #     <visual name="top_visual">
    #   <!-- Lower the mesh by half the height, and rotate by 90 degrees -->
    #       <pose>0 0 -0.0376785 0 0 1.5707</pose>
    #       <geometry>
    #           <mesh>
    #               <uri>model://velodyne_hdl32/meshes/velodyne_top.dae</uri>
    #           </mesh>
    #       </geometry>
    #     </visual>
    def addInertial(self,uri,local_uri,indent_level):
        if local_uri not in self.meshes_inertial_params:
            self.getInertialParams(uri,local_uri)

        self.addOpenTag("inertial",indent_level)
        self.addBreakLine()
        self.addPose(self.meshes_inertial_params[local_uri]["center_mass"],indent_level+1)
        self.addBreakLine()
        self.addOpenTag("mass",indent_level+1)
        self.gazebo_output+=self.meshes_inertial_params[local_uri]["mass"]
        self.addCloseTag("mass")
        self.addBreakLine()
        self.addOpenTag("inertia",indent_level+1)
        self.addBreakLine()
        initial_j=0
        for i in range(0,3):
            for j in range(initial_j,3):
                self.addInertiaComponent(local_uri,i,j,indent_level+2)
                self.addBreakLine()
            initial_j+=1
        self.addCloseTag("inertia",indent_level+1)
        self.addBreakLine()
        self.addCloseTag("inertial",indent_level)


    def addInertiaComponent(self,local_uri,i,j,indent_level):
        tag_name="i"+self.position_to_letter[i]+self.position_to_letter[j]
        self.addOpenTag(tag_name,indent_level)
        self.gazebo_output+=str(self.meshes_inertial_params[local_uri]["inertia"][i][j]*1e03)
        self.addCloseTag(tag_name)


    def addVisualWithMesh(self, name, pose, uri, color, indent_level):

        self.addOpenTag("visual", indent_level, params=[
                        {"name": name+"_visual"}])
        self.addBreakLine()
        # self.addPose(pose, indent_level+1)
        # self.addBreakLine()
        self.addGeometryWithMesh(uri, indent_level+1)
        self.addBreakLine()
        if color[0] != -1:
            color_string = ""
            for c in color:
                color_string += str(c)+" "
            self.addOpenTag("material", indent_level+1)
            self.addBreakLine()
            self.addOpenTag("ambient", indent_level+2)
            self.gazebo_output += color_string
            self.addCloseTag("ambient")
            self.addBreakLine()
            self.addOpenTag("diffuse", indent_level+2)
            self.gazebo_output += color_string
            self.addCloseTag("diffuse")
            self.addBreakLine()
            self.addOpenTag("specular", indent_level+2)
            self.gazebo_output += "0 0 0 0"
            self.addCloseTag("specular")
            self.addBreakLine()
            self.addOpenTag("emissive", indent_level+2)
            self.gazebo_output += "0 0 0 1"
            self.addCloseTag("emissive")
            self.addBreakLine()
            self.addCloseTag("material", indent_level=indent_level+1)
            self.addBreakLine()
        self.addCloseTag("visual", indent_level)

    def addCollisionWithMesh(self, name, pose, uri, indent_level):
        self.addOpenTag("collision", indent_level, params=[
                        {"name": name+"_collision"}])
        self.addBreakLine()
        # self.addPose(pose, indent_level+1)
        # self.addBreakLine()
        self.addGeometryWithMesh(uri, indent_level+1)
        self.addBreakLine()
        self.addOpenTag("surface",indent_level+1)
        self.addBreakLine()
        self.addOpenTag("bounce",indent_level+2)
        self.addCloseTag("bounce")
        self.addBreakLine()        
        self.addOpenTag("friction",indent_level+2)
        self.addBreakLine()
        self.addOpenTag("ode",indent_level+3)
        self.addBreakLine()
        self.addOpenTag("mu",indent_level+4)
        self.gazebo_output+=str(self.friction_coef)
        self.addCloseTag("mu")
        self.addBreakLine()
        self.addOpenTag("mu2",indent_level+4)
        self.gazebo_output+=str(self.friction_coef)
        self.addCloseTag("mu2")
        self.addBreakLine()
        self.addCloseTag("ode",indent_level=indent_level+3)
        self.addBreakLine()
        self.addCloseTag("friction",indent_level+2)
        self.addBreakLine()
        self.addOpenTag("contact",indent_level+2)
        self.addBreakLine()
        self.addOpenTag("ode",indent_level+3)
        self.addCloseTag("ode")
        self.addBreakLine()
        self.addCloseTag("contact",indent_level+2)
        self.addBreakLine()
        self.addCloseTag("surface",indent_level+1)
        self.addBreakLine()
        self.addCloseTag("collision", indent_level)

    def addPose(self, pose, indent_level):
        self.addOpenTag("pose", indent_level)
        self.gazebo_output += pose
        self.addCloseTag("pose")

    def addGeometryWithMesh(self, uri, indent_level):
        self.addOpenTag("geometry", indent_level)
        self.addBreakLine()
        self.addOpenTag("mesh", indent_level+1)
        self.addBreakLine()
        self.addOpenTag("uri", indent_level+2)
        self.gazebo_output += uri
        self.addCloseTag("uri")
        self.addBreakLine()
        self.addCloseTag("mesh", indent_level=indent_level+1)
        self.addBreakLine()
        self.addCloseTag("geometry", indent_level=indent_level)

    def addOpenTag(self, tag_name, indent_level, params=[]):
        self.addIndent(indent_level)
        self.gazebo_output += "<"+tag_name
        for param_dict in params:
            for param_name, param_value in param_dict.items():
                self.gazebo_output += " "+param_name+"='"+param_value+"'"
        self.gazebo_output += ">"
        #print("output after open tag: \n"+self.gazebo_output)

    def addCloseTag(self, tag_name, indent_level=0):
        #print("Output before close tag: "+self.gazebo_output)
        self.addIndent(indent_level)
        self.gazebo_output += "</"+tag_name+">"
        #print("Output after close tag: "+self.gazebo_output)

    def addBreakLine(self):
        #print("Output before break: " +self.gazebo_output)
        self.gazebo_output += "\n"
        #print("Output after break: " +self.gazebo_output)

    def addIndent(self, indent_level):
        #print("Output before indent: \n"+self.gazebo_output)
        for i in range(0, indent_level):
            self.gazebo_output += "\t"
        #print("Output after indent: \n"+self.gazebo_output)

    def initializeGazeboScenario(self, gazebo_output_file_path):
        shutil.copyfile(
            "benchmarkings/lagriffoul/scripts/resources/initial_part_gazebo_world.txt", gazebo_output_file_path)

    def endGazeboScenario(self, gazebo_output_file_path):
        # open both files
        self.gazebo_output += "\n"
        # print(self.gazebo_output)
        with open("benchmarkings/lagriffoul/scripts/resources/ending_part_gazebo_world.txt", 'r') as input_file, open(gazebo_output_file_path, 'a') as output_file:
            output_file.write(self.gazebo_output)
            # read content from first file
            for line in input_file:
                # append content to second file
                output_file.write(line)


    def getInertialParams(self,uri,local_uri):        
        
        mesh=trimesh.load("/ros_ws/src/masther-thesis-miis"+uri)
        #mesh=trimesh.load("/ros_ws/src/masther-thesis-miis"+"/benchmarkings/lagriffoul/problems/pb_3_sorting_objects/meshes/stick_blue.stl")
        self.meshes_inertial_params[local_uri]={"center_mass":str(mesh.center_mass[0])+" "+str(mesh.center_mass[1])+" "+str(mesh.center_mass[2])+" 0.0 0.0 0.0" ,"inertia":mesh.moment_inertia,"mass":str(1000*mesh.volume)}
        print("Mesh: "+local_uri)
        print(self.meshes_inertial_params[local_uri])
        print("volume: "+str(mesh.volume))
        
        print("mass: "+str(mesh.mass))
        #mesh.show()
        # for facet in mesh.facets:
        #     mesh.visual.face_colors[facet] = trimesh.visual.random_color()