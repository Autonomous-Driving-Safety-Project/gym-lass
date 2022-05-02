import os
import math
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import LinearRing


class Utils:
    ROOT_PATH = os.path.join(os.path.dirname(__file__), "..")

    @classmethod
    def load_xosc(cls, path, enable_random=False):
        xosc = ET.parse(path).getroot()
        mins = list()
        maxs = list()
        para_nums = 0
        adjParas = list()
        for i in xosc:
            if i.tag == 'ParameterDeclarations':
                for para in i:
                    # print(para.tag, para.attrib)
                    if 'relativePath' in para.attrib and para.attrib['relativePath'] == "true":
                        para.attrib['value'] = os.path.join(cls.ROOT_PATH, para.attrib['value'])
                    if 'adjustable' in para.attrib and para.attrib['adjustable'] == "true":
                        adjParas.append(para)
                        mins.append(float(para.attrib['min']))
                        maxs.append(float(para.attrib['max']))
                        para_nums += 1
                break
        mins = np.array(mins)
        maxs = np.array(maxs)
        v = np.random.uniform(mins, maxs)

        if enable_random:
            for i in range(0, para_nums):
                adjParas[i].attrib['value'] = str(v[i])

        return ET.tostring(xosc, encoding='unicode', method='xml')

    @classmethod
    def __coordinates(cls, vehicle_state):
        center_x = vehicle_state.x + vehicle_state.center_offset_x
        center_y = vehicle_state.y + vehicle_state.center_offset_y
        width = vehicle_state.width
        length = vehicle_state.length
        heading = vehicle_state.h
        lx = length / 2.0 * math.fabs(math.cos(heading))
        ly = length / 2.0 * math.fabs(math.sin(heading))
        wx = width / 2.0 * math.fabs(math.sin(heading))
        wy = width / 2.0 * math.fabs(math.cos(heading))
        return (center_x + lx + wx, center_y + ly - wy), (center_x - lx + wx, center_y - ly - wy), \
               (center_x - lx - wx, center_y - ly + wy), (center_x + lx - wx, center_y + ly + wy)

    @classmethod
    def gap(cls, vehicle_state_a, vehicle_state_b):
        bounding_box_a = LinearRing(cls.__coordinates(vehicle_state_a))
        bounding_box_b = LinearRing(cls.__coordinates(vehicle_state_b))
        return bounding_box_a.distance(bounding_box_b)
