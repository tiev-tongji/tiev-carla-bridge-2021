#!/usr/bin/env python
# encoding=utf-8

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    B            : toggle hil mode
    O            : toggle data recorder
    M            : toggle manual transmission
    ,/.          : gear up/down

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function
import sys
import json

try:
    import lcm as xcm
    print("---------------USING LCM-----------------")  
    xcm_version = "LCM"
except ImportError:
    try:
        import zerocm as xcm
        print("---------------USING ZCM-----------------")
        xcm_version = "ZCM"
    except ImportError:
        raise RuntimeError(
            'cannot import lcm or zcm, make sure lcm package is installed')

sys.path.append('./tiev2020')
if xcm_version == "LCM":
    try:
        import icumsg_lcm
        from icumsg_lcm import structCANINFO
        # from icumsg_lcm import structCANCONTROLZLG
        from icumsg_lcm import structNAVINFO
        from icumsg_lcm import structFUSIONMAP
        #from icumsg_lcm import structOBJECTLIST
        from icumsg_lcm import structLANES
        # from icumsg_lcm import structNAVINFO
        # from icumsg_lcm import structFUSIONMAP
        # from icumsg_lcm import structCARCONTROL
        # from icumsg_lcm import structAIMPATHINT
        # from icumsg_lcm import structCANCONTROLZLG
        # from icumsg_lcm import structCANINFO
        # from icumsg_lcm import LinePoint, LANE, structLANES
        from icumsg_lcm import structCANCONTROLZLG
        from icumsg_lcm import POSITION, BOUNDINGBOX, OBJECT, structOBJECTLIST
    except ImportError:
        raise RuntimeError("cannot import lcm defination")
elif xcm_version == "ZCM":
    try:
        import icumsg
        from icumsg import structCANINFO
        from icumsg import structCANCONTROLZLG
        from icumsg import structNAVINFO
        from icumsg import structFUSIONMAP
        from icumsg import structOBJECTLIST
        from icumsg import structLANES
        # from icumsg import structNAVINFO
        # from icumsg import structFUSIONMAP
        # from icumsg import structCARCONTROL
        # from icumsg import structAIMPATHINT
        # from icumsg import structCANINFO
        # from icumsg import structCANCONTROLZLG
        # from icumsg import LinePoint, LaneLine, LANE, structLANES
        # from icumsg import POSITION, BOUNDINGBOX, OBJECT, structOBJECTLIST
    except ImportError:
        raise RuntimeError("cannot import zcm defination")


xcm_url = ''
with open("sim_params.json",'r') as load_f:
    tmp = json.load(load_f)
    xcm_url = tmp['xcm_url'][0]  # 0 for IPC on car. 1 for old IPC.
    print(xcm_url)
if xcm_version == "LCM":
    _tunnel = xcm.LCM(xcm_url)
elif xcm_version == "ZCM":
    _tunnel = xcm.ZCM(xcm_url)


sys.path.append('C:\\Users\\autolab\\Desktop\\Carla-0.9.9.4\\PythonAPI\\tiev-carla-sim\\fusionmap_usingcpp\\x64\\Release')
import fusionmap_usingcpp as fmcpp

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os



try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import time
import threading
import json

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_g
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_l
    from pygame.locals import K_i
    from pygame.locals import K_z
    from pygame.locals import K_x
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
    from pygame.locals import K_b
    from pygame.locals import K_o
except ImportError:
    raise RuntimeError(
        'cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

try:
    import pandas as pd
except ImportError:
    raise RuntimeError(
        'cannot import pandas, make sure pandas package is installed')



# try:
#     import zerocm as zcm
# except ImportError:
#     raise RuntimeError('cannot import zerocm, make sure zerocm package is installed')

# try:
#     import icumsg
#     from icumsg import structCANINFO
#     from icumsg import structCANCONTROLZLG
#     from icumsg import structNAVINFO
#     from icumsg import structFUSIONMAP
#     from icumsg import structOBJECTLIST
#     from icumsg import structLANES
# except ImportError:
#     raise RuntimeError('cannot import zerocm messages')

try:
    import utm
except ImportError:
    raise RuntimeError('cannot import utm, make sure utm package is installed')

try:
    from PidController import PID
except ImportError:
    raise RuntimeError('cannot import PidController')

# ==============================================================================
# -- Global functions and vars -------------------------------------------------
# ==============================================================================

# if xcm_version == "LCM":
#     _tunnel = xcm.LCM("udpm://239.255.76.78:7678?ttl=1")
# elif xcm_version == "ZCM":
#     _tunnel = xcm.ZCM("udpm://239.255.76.78:7678?ttl=1")

_navinfo = structNAVINFO()
_caninfo = structCANINFO()
_cancontrolzlg = structCANCONTROLZLG()
# _fusionmap = structFUSIONMAP()
_objectlist = structOBJECTLIST()
_lanes = structLANES()

_mutex_navinfo = threading.Lock()
_mutex_caninfo = threading.Lock()
_mutex_fusionmap = threading.Lock()
_mutex_objectlist = threading.Lock()
_mutex_lanes = threading.Lock()
_mutex_cancontrol = threading.Lock()

_threads = []

_pid_controller = PID()
# 121: 12: 44E 31: 16: 54N
_utm_origin = utm.from_latlon(31.281666666666666, 121.21222222222222)

_record_on = False
_stop_lcm = False
tiev_control = False

kMapRowNum = 501
kMapRowCenter = 351
kMapColNum = 251
kMapColCenter = 126
kMapResolution = 0.2
restart_distance = -1


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters)
               if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


def pub_navinfo_loop(interval):
    while True:
        if _stop_lcm:
            break
        start = time.process_time()
        _mutex_navinfo.acquire()
        if xcm_version == "LCM":
            try:
                _tunnel.publish('NAVINFO', _navinfo.encode())
            except:
                print('LCM:error publishing NAVINFO!')
        elif xcm_version == "ZCM":
            try:
                _tunnel.publish("NAVINFO", _navinfo)
            except:
                print('ZCM:error publishing NAVINFO!')
        _mutex_navinfo.release()
        end = time.process_time()
        sleep_time = interval - end + start
        if sleep_time > 0:
            time.sleep(sleep_time)


def pub_caninfo_loop(interval):
    while True:
        if _stop_lcm:
            break
        start = time.process_time()
        _mutex_caninfo.acquire()
        if xcm_version == "LCM":
            try:
                _tunnel.publish('CANINFO', _caninfo.encode())
            except:
                print('LCM:error publishing CANINFO!')
        elif xcm_version == "ZCM":
            try:
                _tunnel.publish("CANINFO", _caninfo)
            except:
                print('ZCM:error publishing CANINFO!')
        _mutex_caninfo.release()
        end = time.process_time()
        sleep_time = interval - end + start
        if sleep_time > 0:
            time.sleep(sleep_time)


def pub_objectlist_loop(interval):
    while True:
        if _stop_lcm:
            break        
        start = time.process_time()
        _mutex_objectlist.acquire()
        if xcm_version == "LCM":
            try:
                _tunnel.publish('OBJECTLIST', _objectlist.encode())
            except:
                print('LCM:error publishing OBJECTLIST!')
        elif xcm_version == "ZCM":
            try:
                _tunnel.publish("OBJECTLIST", _objectlist)
            except:
                print('ZCM:error publishing OBJECTLIST!')
        _mutex_objectlist.release()
        end = time.process_time()
        sleep_time = interval - end + start
        if sleep_time > 0:
            time.sleep(sleep_time)


def publish_all_async(interval_high, interval_low):
    t_pub_navinfo = threading.Thread(target=pub_navinfo_loop, args=(interval_high,))
    _threads.append(t_pub_navinfo)

    t_pub_caninfo = threading.Thread(target=pub_caninfo_loop, args=(interval_high,))
    _threads.append(t_pub_caninfo)

    t_pub_objectlist = threading.Thread(target=pub_objectlist_loop, args=(interval_low,))
    _threads.append(t_pub_objectlist)

    for thread in _threads:
        thread.start()


def cancontrol_handler(channel, data): 
    msg = structCANCONTROLZLG.decode(data)
    _cancontrolzlg.timestamp = msg.timestamp
    _cancontrolzlg.throttle = msg.throttle
    _cancontrolzlg.steer = msg.steer
    _cancontrolzlg.brake = msg.brake
    _cancontrolzlg.reverse = msg.reverse



def get_xcm():
    while True:
        if _stop_lcm:
            break  
        _tunnel.handle()

def subscribe_all():
    if xcm_version == "LCM":
        _tunnel.subscribe('CANCONTROLZLG', cancontrol_handler)
        thread_sub = threading.Thread(target=get_xcm, args=())
        thread_sub.start()
    elif xcm_version == "ZCM":
        _tunnel.subscribe_raw('CANCONTROL', cancontrol_handler)
        _tunnel.start()


def dot(vector1, vector2):
    return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z


def norm2(x, y, z):
    return math.pow(x * x + y * y + z * z, 0.5)


def to_vehframe(veh_t, loc):
    f = veh_t.get_forward_vector()
    r = veh_t.get_right_vector()
    u = veh_t.get_up_vector()
    rloc = loc - veh_t.location
    x = dot(rloc, r)
    y = dot(rloc, f)
    z = dot(rloc, u)
    return carla.Location(x, y, z)


def to_ue4frame(veh_t, rloc):
    loc = carla.Location(rloc.y, rloc.x, rloc.z)
    veh_t.transform(loc)
    return loc

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print(
                '  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self.camera_manager2 = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._gamma = args.gamma
        self._map = args.map
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.history_position = []

    def restart(self):
        print('world restarting')
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_index2 = 1
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        cam_pos_index2 = 1
        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter("vehicle.mini.cooperst"))
        if self._actor_filter == 'walker':
            blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
            print('walker')
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            #color = random.choice(blueprint.get_attribute('color').recommended_values)
            color = "195,78,78"
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(
                blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(
                blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(
                blueprint.get_attribute('speed').recommended_values[2])
        else:
            pass
            # print("No recommended values for 'speed' attribute")
        # Spawn the player.
        with open("./sim_params.json",'r') as load_f:
            spawn_origin = json.load(load_f)
        if self.player is not None:
            global restart_distance
            spawn_point = self.player.get_transform()
            restart_distance = min(restart_distance, len(self.history_position))
            if len(self.history_position) != 0 and restart_distance != -1:
                spawn_point.location.x = self.history_position[-restart_distance][0]
                spawn_point.location.y = self.history_position[-restart_distance][1]
                spawn_point.location.z = self.history_position[-restart_distance][2]
                spawn_point.rotation.yaw = self.history_position[-restart_distance][3]  
                self.history_position = self.history_position[:-restart_distance]  
            else:
                spawn_point.location.x = spawn_origin[self._map][0][0]
                spawn_point.location.y = spawn_origin[self._map][0][1]
                spawn_point.location.z = spawn_origin[self._map][0][2]
                spawn_point.rotation.yaw = math.degrees(-spawn_origin[self._map][1][0])            
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            restart_distance = -1
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            print('exist {:d} spawn_points'.format(len(spawn_points)))
            spawn_point = random.choice(
                spawn_points) if spawn_points else carla.Transform()
            spawn_point.location.x = spawn_origin[self._map][0][0]
            spawn_point.location.y = spawn_origin[self._map][0][1]
            spawn_point.location.z = spawn_origin[self._map][0][2]
            spawn_point.rotation.yaw = math.degrees(-spawn_origin[self._map][1][0]) 
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            print(self.player)
            if self.player is None:
                print('spawn point is not correct')
                exit(1)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        
        self.camera_manager2 = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager2.transform_index = cam_pos_index2
        self.camera_manager2.set_sensor(cam_index2, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)
        print('world restarted')

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def tick(self, clock):
        self.hud.tick(self, clock)
        self.pack_objectlist()
        if self.player is not None:
            pos = self.player.get_transform()
            if len(self.history_position) == 0:
                self.history_position.append([pos.location.x, pos.location.y, pos.location.z+0.1, pos.rotation.yaw])
            else:
                if (pos.location.x - self.history_position[-1][0])**2 + (pos.location.y - self.history_position[-1][1])**2 > 1:
                    self.history_position.append([pos.location.x, pos.location.y, pos.location.z+0.1, pos.rotation.yaw])
            if len(self.history_position) >= 10000:
                self.history_position = []

    def render(self, display):
        self.camera_manager.render(display)
        self.camera_manager2.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None
        self.camera_manager2.sensor.destroy()
        self.camera_manager2.sensor = None
        self.camera_manager2.index = None


    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        actors = [
            self.camera_manager.sensor,
            self.camera_manager2.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()

    def pack_one_object(self, actor, obj):
        t_actor = actor.get_transform()
        v_actor = actor.get_velocity()
        a_actor = actor.get_acceleration()
        t_ego = self.player.get_transform()
        f = t_actor.get_forward_vector()
        r = t_actor.get_right_vector()

        vf_actor = dot(v_actor, f)
        vr_actor = dot(v_actor, r)
        obj.v = np.linalg.norm((vf_actor, vr_actor))
        obj.v = obj.v if obj.v > 0 else -obj.v

        r_actor = t_actor.rotation
        heading = -r_actor.yaw
        while heading < -180:
            heading = heading + 180
        while heading > 180:
            heading = heading - 360
        obj.theta = math.radians(heading)

        loc = t_actor.location
        t = 0.5
        for i in range(0, 10):
            p = carla.Location()
            p.x = loc.x + i * t * v_actor.x + \
                0.5 * math.pow(i * t, 2) * a_actor.x
            p.y = loc.y + i * t * v_actor.y + \
                0.5 * math.pow(i * t, 2) * a_actor.y
            p.z = loc.z + i * t * v_actor.z + \
                0.5 * math.pow(i * t, 2) * a_actor.z
            p_vf = to_vehframe(t_ego, p)
            pos = POSITION()
            pos.x = p_vf.x
            pos.y = p_vf.y
            obj.path.append(pos)
        obj.pathNum = len(obj.path)

        bb = actor.bounding_box
        obj.width = bb.extent.y * 2
        obj.length = bb.extent.x * 2
        vertexs = []
        #print("bb location: ", bb.location)
        #print("bb extent: ", bb.extent)
        vertexs.append(carla.Location(bb.location.x + bb.extent.x,
                                      bb.location.y + bb.extent.y, bb.location.z))
        vertexs.append(carla.Location(bb.location.x - bb.extent.x,
                                      bb.location.y + bb.extent.y, bb.location.z))
        vertexs.append(carla.Location(bb.location.x - bb.extent.x,
                                      bb.location.y - bb.extent.y, bb.location.z))
        vertexs.append(carla.Location(bb.location.x + bb.extent.x,
                                      bb.location.y - bb.extent.y, bb.location.z))
        vertexs_vf = []
        for vertex in vertexs:
            # print(vertex)
            vertex = t_actor.transform(vertex)
            # print(vertex)
            vertex = to_vehframe(t_ego, vertex)
            # print(vertex)
            vertexs_vf.append(vertex)

        # ordered = []  # right-up, right-bottom, left-bottom, left-up
        # for i in range(4):
        #     if len(ordered) == 0:
        #         ordered.append(i)
        #     elif vertexs_vf[i].x < vertexs_vf[ordered[-1]].x:
        #         ordered.append(i)
        #     else:
        #         ordered.insert(0, i)
        # if vertexs_vf[ordered[0]].y < vertexs_vf[ordered[1]].y:
        #     ordered[0], ordered[1] = ordered[1], ordered[0]
        # if vertexs_vf[ordered[2]].y > vertexs_vf[ordered[3]].y:
        #     ordered[2], ordered[3] = ordered[3], ordered[2]
        # print(ordered)

        # # right-up
        # obj.corners.p1.x = vertexs_vf[ordered[0]].x
        # obj.corners.p1.y = vertexs_vf[ordered[0]].y
        # # right-bottom
        # obj.corners.p2.x = vertexs_vf[ordered[1]].x
        # obj.corners.p2.y = vertexs_vf[ordered[1]].y
        # # left-bottom
        # obj.corners.p3.x = vertexs_vf[ordered[2]].x
        # obj.corners.p3.y = vertexs_vf[ordered[2]].y
        # # left-up
        # obj.corners.p4.x = vertexs_vf[ordered[3]].x
        # obj.corners.p4.y = vertexs_vf[ordered[3]].y
        # print("bb corners p1: ", obj.corners.p1.x, obj.corners.p1.y)
        # print("bb corners p2: ", obj.corners.p2.x, obj.corners.p2.y)
        # print("bb corners p3: ", obj.corners.p3.x, obj.corners.p3.y)
        # print("bb corners p4: ", obj.corners.p4.x, obj.corners.p4.y)

        # right-up
        obj.corners.p1.x = vertexs_vf[0].x
        obj.corners.p1.y = vertexs_vf[0].y
        # right-bottom
        obj.corners.p2.x = vertexs_vf[1].x
        obj.corners.p2.y = vertexs_vf[1].y
        # left-bottom
        obj.corners.p3.x = vertexs_vf[2].x
        obj.corners.p3.y = vertexs_vf[2].y
        # left-up
        obj.corners.p4.x = vertexs_vf[3].x
        obj.corners.p4.y = vertexs_vf[3].y
        # print("bb corners p1: ", obj.corners.p1.x, obj.corners.p1.y)
        # print("bb corners p2: ", obj.corners.p2.x, obj.corners.p2.y)
        # print("bb corners p3: ", obj.corners.p3.x, obj.corners.p3.y)
        # print("bb corners p4: ", obj.corners.p4.x, obj.corners.p4.y)

    def pack_objectlist(self):
        _mutex_objectlist.acquire()

        actors = self.world.get_actors()
        _objectlist.count = 0
        _objectlist.obj.clear()
        _objectlist.data_source = 1
        _objectlist.timestamp = _navinfo.timestamp
        # print("--------------- start to pack objectlist ------------------")
        for index, actor in enumerate(actors):
            obj = OBJECT()
            obj.id = index
            if actor.id == self.player.id:
                continue
            if actor.type_id.startswith('vehicle'):
                if actor.attributes['number_of_wheels'] == 4:
                    #print("add vehicle")
                    obj.obj_type = 0
                elif actor.attributes['number_of_wheels'] == 2:
                    #print("add bike")
                    obj.obj_type = 1
            elif actor.type_id.startswith('walker.pedestrain'):
                #print("add ped")
                obj.obj_type = 2
            else:
                continue

            rloc_actor = to_vehframe(
                self.player.get_transform(), actor.get_location())
            # print(rloc_actor)
            in_vf = rloc_actor.y + 3 <= kMapRowCenter * kMapResolution and \
                rloc_actor.y - 3 >= -(kMapRowNum - kMapRowCenter) * kMapResolution and \
                rloc_actor.x + 3 <= kMapColCenter * kMapResolution and \
                rloc_actor.x - 3 >= -kMapColCenter * kMapResolution and \
                abs(rloc_actor.z) <= 3
            if not in_vf:
                #print("not in veh frame")
                continue

            self.pack_one_object(actor, obj)


            _objectlist.obj.append(obj)
            _objectlist.count += 1
            #print("size: ", len(_objectlist.obj))

        _mutex_objectlist.release()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""

    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self._hil_enabled = False
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_g:
                    world.toggle_radar()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    global restart_distance
                    restart_distance = int(event.key - K_0) * 5
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                    # world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification(
                        "Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec",
                                       world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification(
                        "Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification(
                        "Recording start time is %d" % (world.recording_start))
                elif event.key == K_o and not pygame.key.get_mods & KMOD_CTRL:
                    _record_on = not _record_on
                    world.hud.notification('Data recorder %s' % (
                        'On' if _record_on else 'Off'
                    ))
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        if self._hil_enabled and not self._autopilot_enabled:
                            self._hil_enabled = False
                            world.hud.notification('HIL Mode %s' % (
                                'On' if self._hil_enabled else 'Off'
                            ))
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    elif event.key == K_SPACE and not pygame.key.get_mods() & KMOD_CTRL:
                        if self._autopilot_enabled and not self._hil_enabled:
                            self._autopilot_enabled = False
                            world.player.set_autopilot(self._autopilot_enabled)
                            # world.hud.notification('Carla Autopilot %s' % (
                            #     'On' if self._autopilot_enabled else 'Off'
                            # ))
                        self._hil_enabled = not self._hil_enabled
                        global tiev_control
                        tiev_control = self._hil_enabled
                        world.hud.notification('TievControl Mode %s' % (
                            'On' if self._hil_enabled else 'Off'
                        ))
                    elif event.key ==K_UP or event.key == K_DOWN or event.key == K_LEFT or event.key == K_RIGHT:
                        self._autopilot_enabled = False
                        world.player.set_autopilot(self._autopilot_enabled)
                        self._hil_enabled = False
                        tiev_control = self._hil_enabled
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker

        if not self._autopilot_enabled and not self._hil_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(
                    pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else:  # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else:  # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights:  # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(
                        carla.VehicleLightState(self._lights))
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(
                    pygame.key.get_pressed(), clock.get_time(), world)
            world.player.apply_control(self._control)
        elif self._hil_enabled:
            self._parse_tiev_control()
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.01, 1)
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        # self._control.hand_brake = keys[K_SPACE]

    def _parse_walker_keys(self, keys, milliseconds, world):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed /= 2
        if keys[K_LEFT] or keys[K_a]:
            #self._control.speed = .01
            self._rotation.yaw -= 0.01 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            #self._control.speed = .01
            self._rotation.yaw += 0.01 * milliseconds
        if keys[K_UP] or keys[K_w]:
            #self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
            self._control.speed = 20
        # self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    def _parse_tiev_control(self):
        # aimspeed = -_cancontrol.aimspeed if _cancontrol.aimspeed < 0 else _cancontrol.aimspeed
        # _pid_controller.tick(aimspeed / 3.6 - _caninfo.carspeed / 360, _cancontrol.aimsteer)
        self._control.steer = _cancontrolzlg.steer
        self._control.brake = _cancontrolzlg.brake
        self._control.throttle = _cancontrolzlg.throttle
        self._control.reverse = _cancontrolzlg.reverse

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            # u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'heading: % 15.2f rad'  % _navinfo.mHeading,
            'Acc    : (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            # 'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f, %3.1f)' % (t.location.x, t.location.y, t.location.z)),
            # 'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (_navinfo.mLon, _navinfo.mLat)),
            'UTM :% 24s' % ('(% 7.2f, % 7.2f)' % (_navinfo.utmX, _navinfo.utmY)),
            #'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('TievControl:', tiev_control),
                ('Hand brake:', c.hand_brake),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles),
            '',
            'Note:',
            'Press SPACE to on/off TievCtrl',
            'Press Q to shift reverse gear',
            'Press BACKSPACE to restart',]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            def distance(l): return math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""

    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 *
                    self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(
            carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude

        _mutex_navinfo.acquire()

        t = self._parent.get_transform()
        # print(t)
        v = self._parent.get_velocity()
        va = self._parent.get_angular_velocity()
        a = self._parent.get_acceleration()
        forward = t.get_forward_vector()
        right = t.get_right_vector()

        _navinfo.utmX = _utm_origin[0] + t.location.x
        _navinfo.utmY = _utm_origin[1] - t.location.y
        gps = utm.to_latlon(_navinfo.utmX, _navinfo.utmY,
                            _utm_origin[2], _utm_origin[3])
        _navinfo.mLat = gps[0]
        _navinfo.mLon = gps[1]
        _navinfo.mAlt = t.location.z

        heading = -t.rotation.yaw
        while heading < -180:
            heading = heading + 360
        while heading > 180:
            heading = heading - 360
        _navinfo.mHeading = math.radians(heading)
        _navinfo.mPitch = t.rotation.pitch
        _navinfo.mAngularRateZ = math.radians(-va.z)

        vel_forward = dot(v, forward)
        vel_right = dot(v, right)
        _navinfo.mSpeed3d = norm2(vel_forward, vel_right, 0)
        _navinfo.mSpeed3d = _navinfo.mSpeed3d if _navinfo.mSpeed3d > 0 else -_navinfo.mSpeed3d
        _navinfo.mVe = v.x
        _navinfo.mVn = -v.y

        _navinfo.mAx = dot(a, right)
        _navinfo.mAy = dot(a, forward)

        _navinfo.timestamp = int(event.timestamp * 1000)
        _navinfo.mCurvature = 0
        _navinfo.mRTKStatus = 1
        _navinfo.mHPOSAccuracy = 0.01
        _navinfo.isReckoningVaild = 1
        _navinfo.mGpsNumObs = 11

        _mutex_navinfo.release()

        _mutex_caninfo.acquire()

        _caninfo.timestamp = int(event.timestamp * 1000)
        _caninfo.carspeed = int(_navinfo.mSpeed3d * 3.6 * 100)
        _caninfo.carsteer = int(-self._parent.get_control().steer * 540)

        _mutex_caninfo.release()


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(
                sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(
                sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.velocity_range = 7.5  # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=2.8, z=1.0),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / \
                self.velocity_range  # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        # self.num_points_1D = 30000*3
        # self.points = np.array([0]*self.num_points_1D)
        # self.points_stack_pos = 0
        # self.points = np.zeros((1, 3), dtype=np.dtype('f4'))
        self.points_1d = []
        self.num_points_1d = 50000*3
        self.points_np = np.array([0]*self.num_points_1d,dtype='d')
        self.stack_pos = 0
        self.onProcessing=False
        self.map_processor = fmcpp.DrivableAreaDetector()
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        Attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=0.0, y=0.0, z=2), carla.Rotation(pitch=0.0, roll=0.0, yaw=0.0)), Attachment.Rigid),
            (carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), Attachment.SpringArm)
            # (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
            # (carla.Transform(carla.Location(x=5.5, y=1.5, z=1.5)), Attachment.SpringArm),
            # (carla.Transform(carla.Location(x=-8.0, z=6.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
            # (carla.Transform(carla.Location(x=-1, y=-bound_y, z=0.5)), Attachment.Rigid)
        ]
        self.transform_index = 1
        self.sensors = [
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {
                'range': '20000',
                'rotation_frequency': '20',
                'channels': '64',
                'points_per_second': '800000',
                'upper_fov': '10',
                'lower_fov': '-20'}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth,
                'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw,
                'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
                'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                 'lens_circle_falloff': '3.0',
                 'chromatic_aberration_intensity': '0.5',
                 'chromatic_aberration_offset': '0'}]]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 200
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)
        self.index = None
        self.time_01 = 0
        self.total_time = 0
        self.count_lidar = 0
        self.count_point = 0


    def toggle_camera(self):
        self.transform_index = (self.transform_index +
                                1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index]
                               [2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))
    
    def fusionmap_process(self, useful_points, height_points, pointstmp):
        for point in pointstmp:
            point_tuple = (point[0],point[1])
            if point_tuple in useful_points:
                continue
            if point_tuple not in height_points:
                height_points[point_tuple] = point[2]
                continue
            if abs(height_points[point_tuple] - point[2]) > 0.1:
                useful_points.add(point_tuple)

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.onProcessing:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'): 
            _mutex_fusionmap.acquire()
            points_temp = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            temp_end = int(len(points_temp)/3)*3
            stack_end = self.stack_pos + temp_end
            if stack_end < self.num_points_1d:
                start = time.process_time()
                self.points_np[self.stack_pos: stack_end] = points_temp[:temp_end]
                end = time.process_time()
                self.stack_pos = stack_end
                _mutex_fusionmap.release()
                return
            else:
                stack_end = self.num_points_1d
                self.points_np[self.stack_pos: ] = points_temp[: stack_end - self.stack_pos]
                self.stack_pos = 0
                self.onProcessing = True



            start = time.perf_counter()
            _fusionmap = structFUSIONMAP()
            _fusionmap.cells = []
            _mutex_objectlist.acquire()
            object_list = _objectlist
            _mutex_objectlist.release()
            object_list_pass2cpp = []
            for i in range(object_list.count):
                obj = object_list.obj[i]
                object_list_pass2cpp.append([obj.corners.p2.x, obj.corners.p2.y, obj.corners.p1.x, obj.corners.p1.y, \
                    obj.length, obj.width, 1])
            self.map_processor.GetGridMap(self.points_np, _fusionmap.cells, 0, object_list_pass2cpp, True) # 0 for not showing info. 1 for showing.
            _fusionmap.resolution = kMapResolution
            _fusionmap.cols = kMapColNum
            _fusionmap.rows = kMapRowNum
            _fusionmap.center_col = kMapColCenter
            _fusionmap.center_row = kMapRowCenter
            end = time.perf_counter()
            # print('spend {:.2f} ms to process point cloud'.format((end-start)*1000))
            _mutex_fusionmap.release()
            self.onProcessing = False

            _tunnel.publish('FUSIONMAP', _fusionmap.encode())


#****************************** using cpp(list and vector) *********************************
            # _mutex_fusionmap.acquire()
            # points_temp = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            # start = time.process_time()
            # self.points_1d.extend(points_temp[:int(len(points_temp)/3)*3])
            # if len(self.points_1d) < self.num_points_1d:
            #     _mutex_fusionmap.release()
            #     return
            # self.onProcessing = True

            # _fusionmap = structFUSIONMAP()
            # _fusionmap.cells = []
            # _fusionmap.resolution = kMapResolution
            # _fusionmap.cols = kMapColNum
            # _fusionmap.rows = kMapRowNum
            # _fusionmap.center_col = kMapColCenter
            # _fusionmap.center_row = kMapRowCenter  

            # start = time.process_time()
            # fmcpp.foo(self.points_1d, _fusionmap.cells)
            # self.points_1d = []
            # _mutex_fusionmap.release()
            # self.onProcessing = False
            # end = time.process_time()
            # print('spend ', end-start, ' to process point cloud')
            # start = time.process_time()
            # # _tunnel.publish('FUSIONMAP', _fusionmap.encode())
            # end = time.process_time()
            # print('spend: ', end-start, ' to pack fusionmap')
            

            # _mutex_fusionmap.acquire()
            # start = time.process_time()
            # frame_points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            # frame_points = np.reshape(frame_points, (int(frame_points.shape[0] / 3), 3))
            # self.points = np.vstack((self.points, frame_points))
            # # len_frame_points_1D = int(len(frame_points_1D)/3)*3
            # # if self.points_stack_pos + len_frame_points_1D > self.num_points_1D:
            # #     self.points[self.points_stack_pos:] = frame_points_1D[:self.num_points_1D - self.points_stack_pos]
            # # else:
            # #     self.points[self.points_stack_pos : max(self.points_stack_pos + len_frame_points_1D, self.num_points_1D)] = frame_points_1D
            # # if len_frame_points_1D / 3 != 0:
            # #     print('warning: 长度不为3的倍数')
            # # self.points_stack_pos += len_frame_points_1D
            # end = time.process_time()
            # if end-start > 0.01:
            #     print("spend ", end - start, " to 累计点云")
            # if self.points.shape[0] < 40000:
            #     _mutex_fusionmap.release()
            #     return
            # self.onProcessing = True

# ******************************* using dict ****************************************** #
            # #using dict to process fusionmap
            # #save self.points
            # start = time.process_time()
            # points = self.points
            # self.points = np.zeros((1, 3), dtype=np.dtype('f4'))
            # _mutex_fusionmap.release()

            # # filter points that is $topheight higher than the lidar cause those points are useless.
            # topheight = -0.3 # the z axis in carla is downward.
            # height_filter = (points[:, 2] > topheight)
            # points = points[np.where(height_filter == True)]           
            # end = time.process_time()    
            # print("spend ", end - start, " to filter height")
            # start = time.process_time()

            # # transfter point cloud frame system to gridmap frame system and filter points which are out of gridmap or inside ego car's bounding box.
            # temp_data = np.array(points[:, :2])
            # temp_data *= 1 / kMapResolution
            # temp_data += (kMapColCenter + 0.5, kMapRowCenter + 0.5)
            # temp_data = temp_data.astype(np.int32)
            # # temp_data = np.reshape(temp_data, (-1, 2))
            # ego_bound_x = int(self._parent.bounding_box.extent.x / kMapResolution) + 1
            # ego_bound_y = int(self._parent.bounding_box.extent.y / kMapResolution) + 1
            # indices = ((temp_data[:, 0] >= 0) & (temp_data[:, 0] < kMapColNum) &
            #            (temp_data[:, 1] >= 0) & (temp_data[:, 1] < kMapRowNum) &
            #            (~((temp_data[:, 0] < kMapColCenter + ego_bound_y) & (temp_data[:, 0] >= kMapColCenter - ego_bound_y) &
            #             (temp_data[:, 1] < kMapRowCenter + ego_bound_x) & (temp_data[:, 1] >= kMapRowCenter - ego_bound_x))))
            # temp_data = temp_data[np.where(indices == True)]
            # pointstmp = points[np.where(indices == True)]
            # pointstmp[:, :2] = temp_data[:, :2]
            # end = time.process_time()    
            # print("spend ", end - start, " to transfer frame")
            # start = time.process_time()            

            # # filter points on the ground.
            # useful_points = set() # points to be used for create gridmap
            # height_points = {} # record the height of (x,y) points
            # point_tuple = (0,0)

# #modify for parallel processing
#             # parallel processing gridmap
#             start = time.process_time()
#             num_points = int(len(pointstmp))
#             points_parts = [0, int(num_points/6), int(num_points*2/6), int(num_points*3/6), int(num_points*4/6),int(num_points*5/6),num_points]
#             sub_threads = []
#             for i in range(len(points_parts)-1):
#                 sub_threads.append(threading.Thread(target=self.fusionmap_process, args=(useful_points, height_points, pointstmp[points_parts[i]:points_parts[i+1]])))
#             for i in range(len(sub_threads)):
#                 sub_threads[i].start()
#             for i in range(len(sub_threads)):
#                 sub_threads[i].join()   
#             end = time.process_time()    
#             print("spend ", end - start, " 并行耗时")  
#             _mutex_fusionmap.release()       
# #modify

            # start = time.process_time()
            # for point in pointstmp:
            #     point_tuple = (point[0],point[1])
            #     if point_tuple in useful_points:
            #         continue
            #     if point_tuple not in height_points:
            #         height_points[point_tuple] = point[2]
            #         continue
            #     if abs(height_points[point_tuple] - point[2]) > 0.02:
            #         useful_points.add(point_tuple)
            # end = time.process_time()    
            # print("spend ", end - start, " 循环耗时")
            
            # start = time.process_time()
            # useful_points = np.array(list(useful_points)).astype(int)
            # print('useful_points shape\n',useful_points.shape)
            # end = time.process_time()    
            # print("spend ", end - start, " 转换矩阵")
        
            # start = time.process_time()
            # # save points into the gridmap
            # self.onProcessing = False # reserve some time for the next round of point cloud collection
            # temp_img_size = (kMapColNum, kMapRowNum)
            # temp_img = np.zeros(temp_img_size, dtype=np.int32)
            # temp_img[useful_points[:,0],useful_points[:,1]] = 2
            # temp_img = temp_img.T
            # _fusionmap = structFUSIONMAP()
            # _fusionmap.cells = temp_img.tolist()
            # _fusionmap.resolution = kMapResolution
            # _fusionmap.cols = kMapColNum
            # _fusionmap.rows = kMapRowNum
            # _fusionmap.center_col = kMapColCenter
            # _fusionmap.center_row = kMapRowCenter  
            # _tunnel.publish('FUSIONMAP', _fusionmap.encode())
            # end = time.process_time()    
            # print("spend ", end - start, " to save gridmap")
                   


        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]
                    ['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(
                dvs_img.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            pass
            image.save_to_disk('_out/%08d' % image.frame)

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        #client.load_world('Town04')

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args)
        controller = KeyboardControl(world, args.autopilot)

        publish_all_async(0.02, 0.05)
        subscribe_all()

        clock = pygame.time.Clock()

        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock):
                return
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()
            print('world destroyed !')

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1080x720',
        help='window resolution (default: 800x600)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '-m', '--map',
        metavar='NAME',
        default='Town03',
        help='actor filter (default: "Town04")')      
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    #print(__doc__)
    print('brige running')

    try:
        game_loop(args)
        global _stop_lcm
        _stop_lcm = True
        

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
