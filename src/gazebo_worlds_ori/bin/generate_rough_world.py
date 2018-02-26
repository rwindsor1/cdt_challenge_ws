#!/usr/bin/env python

from __future__ import print_function
from math import log
from math import pow
from math import sqrt
import argparse
from datetime import datetime

import random

import rospkg

from shutil import copyfile
from os import remove

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("sidelength")
    parser.add_argument("complexity")
    parser.add_argument("h")
    parser.add_argument("tiles")
    parser.add_argument("fractal")
    parser.add_argument("parent_frame")
    args = parser.parse_args()
    sidelength = float(args.sidelength)
    complexity = int(args.complexity)
    h = float(args.h)
    tiles = str(args.tiles)
    fractal = str(args.fractal)
    parent_frame = args.parent_frame
    
    print('Generating rough terrain')
    print(' sidelength: ' + str(sidelength) + " [m]")
    print(' with complexity(fractal) / density(random): ' + str(complexity))
    print(' and smoothness(fractal) / sigma(random): ' + str(h))
    print(' using tiles: ' + str(tiles))
    print(' using fractals: ' + str(fractal))
    print(' parent frame: ' + parent_frame)
    
    if tiles == 'true':
        dimension = int(pow(2, complexity) + 1)
    else:
        dimension = int(pow(2, complexity))
    
    # vertices:
    # with tiles= [(dimension) rows: [(dimension) columns: [4 vertices per tile: [p_x, p_y, p_z]]]
    # or smooth = [(dimension + 1) rows: [(dimension + 1) columns: [[p_x, p_y, p_z]]]
    vertices = []
    # normals:
    if tiles == 'true':
        # normals = [6 possible]
        normals = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
    else:
        # normals = [(2 * (dimension-1)^2) normals: [n_x n_y n_z]]
        normals = []
    # triangles:
    # with tiles= [(6 * dimension^2 - 2 * 2 * dimension) triangles: [3 vertex_indices + 3 normal_indices]]
    # or smooth = [(2 * (dimension-1)^2) triangles: [3 vertex_indices + 3 normal_indices]]
    triangles = []
    
    if tiles == 'true':
        print(' -> results in ' + str(dimension * dimension * 4) + ' vertices or '+ str(dimension * dimension) +' tiles')
    else:
        print(' -> results in ' + str((dimension-1) * (dimension-1) * 4) + ' vertices')
    
    generateVertices(dimension, sidelength, vertices, h, tiles, fractal)
    if tiles != 'true':
        generateSmoothNormals(dimension, normals, vertices)
    generateTriangles(dimension, triangles, vertices, tiles)
    colladaWriter(vertices, normals, triangles)
    worldWriter(parent_frame)


def generateVertices(dimension, sidelength, vertices, h, tiles, fractal):
    minimum = -sidelength/2.0
    maximum = sidelength/2.0
    if tiles != 'true':
        dimension += 1
    
    vertices[:] = [None] * (dimension)
    for row in range(dimension):
        vertices[row] = [None] * (dimension)
        for column in range(dimension):
            if tiles == 'true':
                vertices[row][column] = [None] * 4
            else:
                vertices[row][column] = None
    
    values = []
    if tiles == 'true':
        for i in range(dimension + 1):
            values.append(minimum + float(maximum - minimum) / float(dimension) * float(i))
    else:
        for i in range(dimension):
            values.append(minimum + float(maximum - minimum) / float(dimension - 1) * float(i))
    
    z = []
    
    if fractal == 'true':
        generateFractalVerticalValues(dimension, z, h)
    else:
        generateRandomVerticalValues(dimension, z, h)
    
    if tiles == 'true':
        for y in range(dimension):
            for x in range(dimension):
                vertices[y][x][0] = [values[x], values[y], z[x][y]]
                vertices[y][x][1] = [values[x+1], values[y], z[x][y]]
                vertices[y][x][2] = [values[x], values[y+1], z[x][y]]
                vertices[y][x][3] = [values[x+1], values[y+1], z[x][y]]
    else:
        for y in range(dimension):
            for x in range(dimension):
                vertices[y][x] = [values[x], values[y], z[x][y]]

def generateRandomVerticalValues(dimension, z, sigma):
    mu = 0.0
    for x in range(dimension):
        zRow = []
        for y in range(dimension):
            zRow.append(random.normalvariate(mu, sigma))
        z.append(zRow)


def generateFractalVerticalValues(dimension, z, h):
    if ((dimension-1) & (dimension - 2)):
        print('Fractal terrain generation needs a size equalt to the  power of 2')
        print('  dimension was ' + str(dimension))
        return
    
    for x in range(dimension):
        zRow = []
        for y in range(dimension):
            zRow.append(0.0)
        z.append(zRow)
    
    numberOfIterations = int(log(dimension - 1, 2))
    for i in range(numberOfIterations):
        diamondStep(z, i + 1, h)
        squareStep(z, i + 1, h)


def diamondStep(z, depth, h):
    numberOfSegments = 1 << (depth - 1)
    span = (len(z) - 1) / numberOfSegments
    y = 0
    while y < len(z) - 1:
        x = 0
        while x < len(z) - 1:
            z[x + span / 2][y + span / 2] = (
              z[x][y] 
            + z[x + span][y] 
            + z[x][y + span] 
            + z[x + span][y + span]
            ) / 4.0 \
            + pow(-1,int(random.random()>0.5)) * pow(2, (-h * depth)) * random.random()
            x += span
        y += span


def squareStep(z, depth, h):
    numberOfSegments = 1 << (depth - 1)
    span = (len(z) - 1) / numberOfSegments
    y = 0
    while y < len(z) - 1:
        x = 0
        while x < len(z) - 1:
            z[x + span/2][y] = (
              z[x][y] 
            + z[(x + span/2)%len(z)][(y + span/2)%len(z)] 
            + z[(x + span)%len(z)][y] 
            + z[(x + span/2)%len(z)][(y - span/2)%len(z)]) \
            / 4.0 + pow(-1,int(random.random()>0.5)) * pow(2, (-h * depth)) * random.random()
            z[x][y + span/2] = (
              z[x][y]
            + z[(x + span/2)%len(z)][(y + span/2)%len(z)]
            + z[x][(y + span)%len(z)]
            + z[(x - span/2)%len(z)][(y + span/2)%len(z)]) \
            / 4.0 + pow(-1,int(random.random()>0.5)) * pow(2, (-h * depth)) * random.random()
            if x == 0:
                z[len(z) - 1][y + span/2] = z[x][y + span/2]
            if y == 0:
                z[x + span/2][len(z) - 1] = z[x + span/2][y]
            x += span
        y += span


def generateSmoothNormals(dimension, normals, vertices):
    for y in range(dimension):
        for x in range(dimension):
            ba = []
            ca = []
            da = []
            for i in range(3):
                ba += [vertices[y][x+1][i] - vertices[y][x][i]]
                ca += [vertices[y+1][x][i] - vertices[y][x][i]]
                da += [vertices[y+1][x+1][i] - vertices[y][x][i]]
            crossProduct = [ba[1] * da[2] - ba[2] * da[1], 
                            -1 * (ba[0] * da[2] - ba[2] * da[0]), 
                            ba[0] * da[1] - ba[1] * da[0]]
            crossProduct += [da[1] * ca[2] - da[2] * ca[1], 
                            -1 * (da[0] * ca[2] - da[2] * ca[0]), 
                            da[0] * ca[1] - da[1] * ca[0]]
            length = [0.0, 0.0]
            for i in range(6):
                length[i/3] += crossProduct[i]*crossProduct[i]
            length[0] = sqrt(length[0])
            length[1] = sqrt(length[1])
            for i in range(6):
                crossProduct[i] = crossProduct[i]/length[i/3]
            normals.append(crossProduct[:3])
            normals.append(crossProduct[3:])


def generateTriangles(dimension, triangles, vertices, tiles):
    # dimension is the number of points for one side
    # for each line and each (dimension-1) points we create two triangles
    #    dimension = 3:
    # 2 ___ ___ 
    #  |  /|  /|
    #  | / | / |
    # 1|/__|/__|
    #  |  /|  /|
    #  | / | / |
    # 0|/__|/__|
    #  0   1   2
    
    i = 0
    normalIndex = 0
    if tiles == 'true':
        for tileIndex in range(dimension * dimension):
            i = tileIndex * 4
            # Two triangles for tile top
            normalIndex = 4
            triangles.append([i, normalIndex, i + 1, normalIndex, i + 3, normalIndex])
            triangles.append([i, normalIndex, i + 3, normalIndex, i + 2, normalIndex])
            # Two triangles for right side
            if (tileIndex % dimension) != dimension - 1:
                if vertices[tileIndex / dimension][tileIndex % dimension][0][2] >= \
                   vertices[(tileIndex+1)/dimension][(tileIndex+1) % dimension][0][2]:
                    normalIndex = 2
                else:
                    normalIndex = 3
                triangles.append([i + 1, normalIndex, i + 4, normalIndex, i + 3, normalIndex])
                triangles.append([i + 3, normalIndex, i + 4, normalIndex, i + 4 + 2, normalIndex])
            # Two triangles for upper side
            if tileIndex < dimension * (dimension - 1):
                if vertices[tileIndex / dimension][tileIndex % dimension][0][2] >= \
                   vertices[(tileIndex+dimension)/dimension][(tileIndex+dimension) % dimension][0][2]:
                    normalIndex = 1
                else:
                    normalIndex = 0
                triangles.append([i + 3, normalIndex, i + 4 * dimension + 1, normalIndex, i + 2, normalIndex])
                triangles.append([i + 2, normalIndex, i + 4 * dimension + 1, normalIndex, i + 4 * dimension, normalIndex])
    else:
        for i in range((dimension+1) * (dimension)):
            if (i % (dimension+1)) != dimension:
                triangles.append([i, normalIndex, i + 1, normalIndex, i + dimension + 2, normalIndex])
                normalIndex += 1
                triangles.append([i, normalIndex, i + dimension + 2, normalIndex, i + dimension + 1, normalIndex])
                normalIndex += 1

def colladaWriter(vertices, normals, triangles):
    datetime.now()
    vertices_length = 0
    vertices_string = '            '
    for row in vertices:
        for column in row:
            if isinstance(column[0], list):
                for vertex in column:
                    for value in vertex:
                        vertices_string += str(value) + ' '
                        vertices_length += 1
                    vertices_string += '\n' + '            '
            else:
                for value in column:
                    vertices_string += str(value) + ' '
                    vertices_length += 1
                vertices_string += '\n' + '            '
    
    normals_string = '            '
    for normal in normals:
        for value in normal:
            normals_string += str(value) + ' '
        normals_string += '\n' + '            '
    
    triangles_string = '            '
    for triangle in triangles:
        for value in triangle:
            triangles_string += str(value) + ' '
        triangles_string += '\n' + '            '
        
    colladaFileName = 'random_rough_surface.dae'
    colladaFileHandle = open(colladaFileName,'w')
    colladaFileHandle.write( \
    '<?xml version="1.0" encoding="utf-8"?>\n' \
    '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">\n' \
    '  <asset>\n' \
    '    <contributor>\n' \
    '      <author>LI</author>\n' \
    '      <authoring_tool>rough_surface_generator</authoring_tool>\n' \
    '    </contributor>\n' \
    '    <created>'+ datetime.now().strftime("%Y-%m-%dT%X") +'</created>\n' \
    '    <modified>'+ datetime.now().strftime("%Y-%m-%dT%X") +'</modified>\n' \
    '    <unit name="meter" meter="1"/>\n' \
    '    <up_axis>Z_UP</up_axis>\n' \
    '  </asset>\n' \
    '  <library_images/>\n' \
    '  <library_geometries>\n' \
    '    <geometry id="mesh" name="rough-mesh">\n' \
    '      <mesh>\n' \
    '        <source id="mesh-positions">\n' \
    '          <float_array id="mesh-positions-array" count="' + str( vertices_length ) + '">\n' \
    + vertices_string[:-12] + \
    '          </float_array>\n' \
    '          <technique_common>\n' \
    '            <accessor source="#mesh-positions-array" count="' + str( vertices_length / 3 ) + '" stride="3">\n' \
    '              <param name="X" type="float"/>\n' \
    '              <param name="Y" type="float"/>\n' \
    '              <param name="Z" type="float"/>\n' \
    '            </accessor>\n' \
    '          </technique_common>\n' \
    '        </source>\n' \
    '        <source id="mesh-normals">\n' \
    '          <float_array id="mesh-normals-array" count="' + str( len(normals) * 3 ) + '">\n' \
    + normals_string[:-12] + \
    '          </float_array>\n' \
    '          <technique_common>\n' \
    '          <accessor source="#mesh-normals-array" count="' + str( len(normals) ) + '" stride="3"> \n' \
    '            <param name="X" type="float"/>\n' \
    '            <param name="Y" type="float"/>\n' \
    '            <param name="Z" type="float"/>\n' \
    '          </accessor>\n' \
    '          </technique_common>\n' \
    '          </source>\n' \
    '          <vertices id="mesh-vertices">\n' \
    '            <input semantic="POSITION" source="#mesh-positions"/>\n' \
    '          </vertices>\n' \
    '        <triangles count="' + str( len(triangles) ) + '">\n' \
    '          <input semantic="VERTEX" source="#mesh-vertices" offset="0"/>\n' \
    '          <input semantic="NORMAL" source="#mesh-normals" offset="1"/>\n' \
    '          <p>\n' \
    + triangles_string[:-12] + \
    '          </p>\n' \
    '        </triangles>\n' \
    '      </mesh>\n' \
    '    </geometry>\n' \
    '  </library_geometries>\n' \
    '    <library_controllers/>\n' \
    '    <library_visual_scenes>\n' \
    '      <visual_scene id="Scene" name="Scene">\n' \
    '        <node id="Plane" name="Plane" type="NODE">\n' \
    '          <matrix sid="transform">1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1</matrix>\n' \
    '          <instance_geometry url="#mesh" name="Plane"/>\n' \
    '        </node>\n' \
    '      </visual_scene>\n' \
    '    </library_visual_scenes>\n' \
    '    <scene>\n' \
    '      <instance_visual_scene url="#Scene"/>\n' \
    '   </scene>\n' \
    '</COLLADA>\n')
    colladaFileHandle.close()

    r = rospkg.RosPack()
    gazeboWorldsPath = r.get_path('gazebo_worlds_ori')
    copyfile(colladaFileName, gazeboWorldsPath + '/meshes/' + colladaFileName)
    # copyfile(colladaFileName, '/home/linus/Desktop/' + colladaFileName)
    remove(colladaFileName)


def worldWriter(parent_frame):
    
    gazeboWorldFileName = 'rough.world'
    gazeboModelFileName = 'model.sdf'
    gazeboConfigFileName = 'model.config'
    yamlFileName = 'rough.yaml'
    
    modelFileHandle = open(gazeboModelFileName,'w')
    modelFileHandle.write( \
    '<?xml version="1.0"?>\n' \
    '<sdf version="1.4">\n' \
    '  <model name="rough">\n' \
    '    <pose>0 0 0 0 0 0</pose>\n' \
    '    <static>true</static>\n' \
    '    <link name="rough_link">\n' \
    '      <collision name="rough_collision">\n' \
    '        <pose>0 0 0 0 0 0</pose>\n' \
    '        <geometry>\n' \
    '          <mesh>\n' \
    '            <uri>model://gazebo_worlds_ori/meshes/random_rough_surface.dae</uri>\n' \
    '          </mesh>\n' \
    '        </geometry>\n' \
    '      </collision>\n' \
    '      <visual name="rough_visual">\n' \
    '        <pose>0 0 0 0 0 0</pose>\n' \
    '        <geometry>\n' \
    '          <mesh>\n' \
    '            <uri>model://gazebo_worlds_ori/meshes/random_rough_surface.dae</uri>\n' \
    '          </mesh>\n' \
    '        </geometry>\n' \
    '      </visual>\n' \
    '    </link> \n' \
    '  </model>\n' \
    '</sdf>\n')
    modelFileHandle.close()
    
    configFileHandle = open(gazeboConfigFileName,'w')
    configFileHandle.write( \
    "<?xml version='1.0'?>\n" \
    '<model>\n' \
    '  <name>rough</name>\n' \
    '  <version>0.1.0</version>\n' \
    '  <sdf>model.sdf</sdf>\n' \
    '  <author>\n' \
    '    <name>Linus Isler</name>\n' \
    '    <email>liisler@mavt.ethz.ch</email>\n' \
    '  </author>\n' \
    '  <description>\n' \
    '    Randomly generated rough surface.\n' \
    '  </description>\n' \
    '</model>\n')
    configFileHandle.close()
    
    worldFileHandle = open(gazeboWorldFileName,'w')
    worldFileHandle.write( \
    '<?xml version="1.0" ?>\n' \
    '<sdf version="1.4">\n' \
    '  <world name="default">\n' \
    '    <include>\n' \
    '      <uri>model://sun</uri>\n' \
    '    </include>\n' \
    '    <include>\n' \
    '      <uri>model://gazebo_worlds_ori/models/rough</uri>\n' \
    '    </include>\n' \
    '    <physics type="ode">\n' \
    '      <gravity>0 0 -9.81</gravity>\n' \
    '    </physics>\n' \
    '  </world>\n' \
    '</sdf>\n')
    worldFileHandle.close()
        
    yamlFileHandle = open(yamlFileName,'w')
    yamlFileHandle.write(
        'models:\n' \
        ' - mesh:\n' \
        '    name: "rough_terrain"\n' \
        '    frame_id: "' + parent_frame + '"\n' \
        '    mesh_resource: "package://gazebo_worlds_ori/meshes/random_rough_surface.dae"\n' \
        '    position:\n' \
        '      x: 0.0 \n' \
        '      y: 0.0\n' \
        '      z: 0.0 \n' \
        '    orientation:\n' \
        '      w: 1.0\n' \
        '      x: 0.0\n' \
        '      y: 0.0\n' \
        '      z: 0.0\n' \
        '    scale:\n' \
        '      x: 1.0\n' \
        '      y: 1.0\n' \
        '      z: 1.0\n')
    yamlFileHandle.close()

    r = rospkg.RosPack()
    gazeboWorldsPath = r.get_path('gazebo_worlds_ori')
    rvizConfigPath = r.get_path('gazebo_worlds_ori')

    copyfile(gazeboWorldFileName, gazeboWorldsPath + '/worlds/' + gazeboWorldFileName)
    copyfile(gazeboModelFileName, gazeboWorldsPath + '/models/rough/' + gazeboModelFileName)
    copyfile(gazeboConfigFileName, gazeboWorldsPath + '/models/rough/' + gazeboConfigFileName)
    copyfile(yamlFileName, rvizConfigPath + '/rviz/' + yamlFileName)
    
    remove(gazeboWorldFileName)
    remove(yamlFileName)


if __name__ == '__main__':
    main()

