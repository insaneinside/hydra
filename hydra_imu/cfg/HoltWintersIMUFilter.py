#!/usr/bin/env python

from __future__ import print_function

import os
import re
from dynamic_reconfigure.parameter_generator_catkin import *

PACKAGE=os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
NODE_NAME=os.path.basename(__file__).partition('.')[0]

words = NODE_NAME.split('_')
title_case_node_name = ''
for w in words:
    if w.islower():
        title_case_node_name += w.capitalize()
    else:
        title_case_node_name += w

gen = ParameterGenerator()

# ex. enumeration
# modes = gen.enum([gen.const('rate', int_t, 0, 'Update at a fixed rate'),
#                   gen.const('event', int_t, 1, 'Update when values change')],
#                  'Possible values for the `mode` parameter')
# add with gen.add(..., edit_method=modes)


class ParameterInfo(object):
    def __init__(self, _name, _type, _description, _default, _min, _max):
        self.name = _name
        self.type = _type
        self.description = _description
        self.default = _default
        self.min = _min
        self.max = _max

parameters = [# ParameterInfo('order', int_t, 'Number of times to apply the smoothing filter', 1, 0, 32),
              ParameterInfo('alpha', double_t, 'Data-smoothing factor', 0.5, 0, 1),
              ParameterInfo('beta', double_t, 'Trend-smoothing factor', 0.25, 0, 1)]

level = 1
for measurement in ['linear_acceleration', 'angular_velocity', 'magnetic_field']:
    for param in parameters:
        gen.add('%s_%s'%(measurement, param.name), param.type, level, param.description,
                param.default, param.min, param.max)
        level <<= 1

exit(gen.generate(PACKAGE, PACKAGE + '/' + NODE_NAME, title_case_node_name))
