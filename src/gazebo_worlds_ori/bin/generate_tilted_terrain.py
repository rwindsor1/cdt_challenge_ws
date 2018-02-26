#!/usr/bin/env python

from __future__ import print_function
from math import sin
from math import cos
import argparse

import rospkg

from shutil import copyfile
from os import remove

def writer():

    parser = argparse.ArgumentParser()
    parser.add_argument("slope")
    parser.add_argument("l1")
    parser.add_argument("l2")
    parser.add_argument("x0")
    parser.add_argument("parent_frame")
    args = parser.parse_args()

    print('Generating a tilted terrain with the following attributes:')
    print('slope: ' + args.slope)
    print('slope length: ' + args.l1)
    print('platform length: ' + args.l2)
    print('terrain offset: ' + args.x0)
    print('parent frame: ' + args.parent_frame)

    parent_frame = args.parent_frame

    slope = float(args.slope)
    l1 = float(args.l1)
    l2 = float(args.l2)
    x0 = float(args.x0)

    x0_hat = x0 + 0.5*l1*cos(slope);
    x1_hat = x0 + l1*cos(slope) + 0.5*l2;
    x2_hat = x0 + 1.5*l1*cos(slope) +l2;

    z0_hat = 0.5*l1*sin(slope);
    z1_hat = l1*sin(slope);
    z2_hat = 0.5*l1*sin(slope);

    gazeboFileName = 'slope.world'
    yamlFileName = 'slope.yaml'

    worldFileHandle = open(gazeboFileName,'w')
    worldFileHandle.write( \
    '<?xml version="1.0" ?>\n' \
    '<sdf version="1.4">\n' \
    '  <world name="default">\n' \
    '    <include>\n' \
    '      <uri>model://ground_plane</uri>\n' \
    '    </include>\n' \
    '    <include>\n' \
    '      <uri>model://sun</uri>\n' \
    '    </include>\n' \
    '    <model name="slope_up">\n' \
    '      <static>true</static>\n' \
    '      <pose>' + str(x0_hat) + ' 0 ' + str(z0_hat) + ' 0 ' + str(-slope) + ' 0</pose>\n' \
    '      <link name="link">\n' \
    '        <collision name="collision">\n' \
    '          <geometry>\n' \
    '            <box>\n' \
    '              <size>' + str(l1) + ' 2 0.01</size>\n' \
    '            </box>\n' \
    '          </geometry>\n' \
    '        </collision>\n' \
    '        <visual name="visual">\n' \
    '          <geometry>\n' \
    '            <box>\n' \
    '              <size>' + str(l1) + ' 2 0.01</size>\n' \
    '            </box>\n' \
    '          </geometry>\n' \
    '        </visual>\n' \
    '      </link>\n' \
    '    </model>\n' \
    '    <model name="platform">\n' \
    '      <static>true</static>\n' \
    '      <pose>' + str(x1_hat) + ' 0 ' + str(z1_hat) + ' 0 0 0</pose>\n' \
    '      <link name="link">\n' \
    '        <collision name="collision">\n' \
    '          <geometry>\n' \
    '            <box>\n' \
    '              <size>' + str(l2) + ' 2 0.01</size>\n' \
    '            </box>\n' \
    '          </geometry>\n' \
    '        </collision>\n' \
    '        <visual name="visual">\n' \
    '          <geometry>\n' \
    '            <box>\n' \
    '              <size>' + str(l2) + ' 2 0.01</size>\n' \
    '            </box>\n' \
    '          </geometry>\n' \
    '        </visual>\n' \
    '      </link>\n' \
    '    </model>\n' \
    '    <model name="slope_down">\n' \
    '      <static>true</static>\n' \
    '      <pose>' + str(x2_hat) + ' 0 ' + str(z2_hat) + ' 0 ' + str(slope) + ' 0</pose>\n' \
    '      <link name="link">\n' \
    '        <collision name="collision">\n' \
    '          <geometry>\n' \
    '            <box>\n' \
    '              <size>' + str(l1) + ' 2 0.01</size>\n' \
    '            </box>\n' \
    '          </geometry>\n' \
    '        </collision>\n' \
    '        <visual name="visual">\n' \
    '          <geometry>\n' \
    '            <box>\n' \
    '              <size>' + str(l1) + ' 2 0.01</size>\n' \
    '            </box>\n' \
    '          </geometry>\n' \
    '        </visual>\n' \
    '      </link>\n' \
    '    </model>\n' \
    '  </world>\n' \
    '</sdf>\n')
    worldFileHandle.close()

    qw = cos(slope/2)
    qvx = 0
    qvy = sin(slope/2)
    qvz = 0

    yamlFileHandle = open(yamlFileName,'w')
    yamlFileHandle.write(\
    'models:\n' \
    ' - box:\n' \
    '    name: "slope_up"\n' \
    '    frame_id: "' + parent_frame + '"\n' \
    '    position:\n' \
    '      x: ' + str(x0_hat) + ' \n' \
    '      y: 0.0\n' \
    '      z: ' + str(z0_hat) + ' \n' \
    '    orientation:\n' \
    '      w: ' + str(qw) + '\n' \
    '      x: ' + str(-qvx) + '\n' \
    '      y: ' + str(-qvy) + '\n' \
    '      z: ' + str(-qvz) + '\n' \
    '    scale:\n' \
    '      x: ' + str(l1) + '\n' \
    '      y: 2\n' \
    '      z: 0.01\n' \
    ' - box:\n' \
    '    name: "platform"\n' \
    '    frame_id: "' + parent_frame + '"\n' \
    '    position:\n' \
    '      x: ' + str(x1_hat) + '\n' \
    '      y: 0.0\n' \
    '      z: ' + str(z1_hat) + '\n' \
    '    orientation:\n' \
    '      w: 1\n' \
    '      x: 0\n' \
    '      y: 0\n' \
    '      z: 0\n' \
    '    scale:\n' \
    '      x: ' + str(l2) + '\n' \
    '      y: 2\n' \
    '      z: 0.01\n' \
    ' - box:\n' \
    '    name: "slope_down"\n' \
    '    frame_id: "' + parent_frame + '"\n' \
    '    position:\n' \
    '      x: ' + str(x2_hat) + '\n' \
    '      y: 0.0\n' \
    '      z: ' + str(z2_hat) + '\n' \
    '    orientation:\n' \
    '      w: ' + str(qw) + '\n' \
    '      x: ' + str(qvx) + '\n' \
    '      y: ' + str(qvy) + '\n' \
    '      z: ' + str(qvz) + '\n' \
    '    scale:\n' \
    '      x: ' + str(l1) + '\n' \
    '      y: 2\n' \
    '      z: 0.01\n')
    yamlFileHandle.close()

    r = rospkg.RosPack()
    gazeboWorldsPath = r.get_path('gazebo_worlds_ori')
    rvizConfigPath = r.get_path('gazebo_worlds_ori')
    
    copyfile(yamlFileName, rvizConfigPath + '/rviz/' + yamlFileName)
    copyfile(gazeboFileName, gazeboWorldsPath + '/worlds/' + gazeboFileName)

    remove(gazeboFileName)
    remove(yamlFileName)

if __name__ == '__main__':
    writer()

