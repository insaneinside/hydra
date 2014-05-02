#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vi:ts=4 sw=4 et

from __future__ import division
from __future__ import print_function

import os
import sys

# Websocket import
from ws4py.client.threadedclient import WebSocketClient

import json

# Joystick import
import sys
import pygame
from pygame.locals import *

USAGE_STRING = 'Usage: %s HOSTNAME [PORT]' % os.path.basename(__file__)
USE_XY_DIRECT = True


# WebSocket Class
class WebClient(WebSocketClient):
    def opened(self):
        def data_provider():
            #not sure when if ever this is actually called
            for i in range(0, 200, 25):
                print(i)
        def closed(soelf, code, reason=None):
            print("Closed down", code, reason)

        def received_message(self, m):
            print(m)
            if len(m) == 175:
                self.close(reason='Bye bye')

def joy2value(value, halfcontrol=False):
    if halfcontrol:
        value = value / 2.0
    if (abs(value - 0) < 0.05):
        value = 0
    return value


def make_power_levels_msg(joysticks):
    if len(joysticks) >= 2:
        return { 'left': joy2value(joysticks[0].get_axis(1)),
                 'right': joy2value(joysticks[1].get_axis(1)) }
    elif USE_XY_DIRECT:
        return { 'left': joy2value(joysticks[0].get_axis(0)),
                 'right': -joy2value(joysticks[0].get_axis(1)) }
    else:
        js = joysticks[0]
        return { 'left': joy2value(js.get_axis(1), js.get_button(7)),
                 'right': -joy2value(js.get_axis(3)) }



# Main
if __name__ == '__main__':
    try:
        hostname = None
        port = 8080

        if len(sys.argv) > 1:
            hostname = sys.argv[1]
        else:
            raise ValueError(USAGE_STRING)

        if len(sys.argv) > 2:
            port = sys.argv[2]

        # Joystick
        pygame.joystick.init()
        pygame.display.init()

        joysticks = [pygame.joystick.Joystick(i) for i in xrange(pygame.joystick.get_count())]
        for js in joysticks:
            js.init()

        if len(joysticks) == 1 and joysticks[0].get_name().index('Dual Action') >= 0:
            USE_XY_DIRECT = False
        print("Main Joystick is: " + joysticks[0].get_name() + " with " + str(joysticks[0].get_numaxes()) + " axes.")

        # Websocket
        ws = WebClient(('ws://' + hostname + ":" + port + "/"), protocols=['http-only', 'chat'])
        ws.connect()

        ws.send(json.dumps({'op': 'advertise', 'topic': 'joystick', 'type': 'hydra_drive/PowerLevels'}))

        while True:
            pygame.event.pump()
            ws.send(json.dumps({'op': 'publish', 'topic': 'joystick',
                                'msg': make_power_levels_msg(joysticks)}))

            #ws.run_forever()
            pygame.time.wait(15)
    except KeyboardInterrupt:
        ws.close()
        pygame.quit()
