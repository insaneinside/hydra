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

gen.add('gain', double_t, 1, 'AHRS algorithm gain', 0.125, 0, 1)
exit(gen.generate(PACKAGE, PACKAGE + '/' + NODE_NAME, title_case_node_name))
