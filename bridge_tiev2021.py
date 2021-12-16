#!/usr/bin/env python
# encoding=utf-8
"""
bridge for tiev and carla in windows
"""
from __future__ import print_function
import sys
import json

import lcm as xcm 
xcm_version = "LCM"
sys.path.append('./tiev2020')
try:
    import icumsg_lcm
    from icumsg_lcm import structCANINFO
    from icumsg_lcm import structCANCONTROL
    from icumsg_lcm import structNAVINFO
    from icumsg_lcm import structFUSIONMAP
    #from icumsg_lcm import structOBJECTLIST
    from icumsg_lcm import structLANES
    from icumsg_lcm import structCANCONTROLZLG
    # from icumsg_lcm import structNAVINFO
    # from icumsg_lcm import structFUSIONMAP
    # from icumsg_lcm import structCARCONTROL
    # from icumsg_lcm import structAIMPATHINT
    # from icumsg_lcm import structCANCONTROL
    # from icumsg_lcm import structCANINFO
    # from icumsg_lcm import LinePoint, LANE, structLANES
    from icumsg_lcm import POSITION, BOUNDINGBOX, OBJECT, structOBJECTLIST
except ImportError:
    raise RuntimeError("cannot import lcm defination")
xcm_url = "udpm://239.255.76.67:7667?ttl=1"
if xcm_version == "LCM":
    _tunnel = xcm.LCM(xcm_url)
elif xcm_version == "ZCM":
    _tunnel = xcm.ZCM(xcm_url)

# fusionmap
sys.path.append('./fusionmap_usingcpp/x64/Release')
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
import math

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_1
    from pygame.locals import K_2
    from pygame.locals import K_3
    from pygame.locals import K_4
    from pygame.locals import K_5
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
    from pygame.locals import K_DELETE
    from pygame.locals import K_LEFTBRACKET
    from pygame.locals import K_RIGHTBRACKET
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
using_zlg_control = False
if not using_zlg_control:
    _cancontrol = structCANCONTROL()
else:
    _cancontrol = structCANCONTROLZLG()

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
print("origin: ", _utm_origin[0], ", ", _utm_origin[1])
#_utm_origin = utm.from_latlon(31.281589, 121.215578)

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
    msg = structCANCONTROL.decode(data)
    _cancontrol.aimsteer = msg.aimsteer  # deg
    _cancontrol.aimspeed = msg.aimspeed

def cancontrol_handler_zlg(channel, data): 
    msg = structCANCONTROLZLG.decode(data)
    _cancontrol.timestamp = msg.timestamp
    _cancontrol.throttle = msg.throttle
    _cancontrol.steer = msg.steer
    _cancontrol.brake = msg.brake
    _cancontrol.reverse = msg.reverse


def get_xcm():
    while True:
        if _stop_lcm:
            break  
        _tunnel.handle()

global _tunnel_sub
if using_zlg_control:
    _tunnel_sub = _tunnel.subscribe('CANCONTROLZLG', cancontrol_handler_zlg)
else:
    _tunnel_sub = _tunnel.subscribe('CANCONTROL', cancontrol_handler)
def subscribe_all():
    if xcm_version == "LCM":
        if using_zlg_control:
            _tunnel.subscribe('CANCONTROLZLG', cancontrol_handler_zlg)
            thread_sub = threading.Thread(target=get_xcm, args=())
            thread_sub.start()
        else:
            _tunnel.subscribe('CANCONTROL', cancontrol_handler)
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

#传入点云对象
def points2pcd(points, fileid):
	#存放路径
    PCD_DIR_PATH=os.path.join(os.path.abspath('E:/huangchang/SFND_Lidar_Obstacle_Detection'),'pcd_files')
    filename = str(fileid) + r'.pcd'
    PCD_FILE_PATH=os.path.join(PCD_DIR_PATH,filename)
    if os.path.exists(PCD_FILE_PATH):
    	os.remove(PCD_FILE_PATH)
    
    #写文件句柄
    handle = open(PCD_FILE_PATH, 'w')
    
    #得到点云点数
    point_num=points.shape[0]

    #pcd头部（重要）
    handle.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1')
    string = '\nWIDTH ' + str(point_num)
    handle.write(string)
    handle.write('\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0')
    string = '\nPOINTS ' + str(point_num)
    handle.write(string)
    handle.write('\nDATA ascii')

    # 依次写入点
    for i in range(point_num):
        string = '\n' + str(points[i, 0]) + ' ' + str(points[i, 1]) + ' ' + str(points[i, 2]) + ' 1' 
        handle.write(string)
    handle.close()


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
        self.restart(0)
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.history_position = []

    def restart(self, restart_pos_id):
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
        restart_pos_str = "origin_" + str(restart_pos_id)
        if restart_pos_id < 0:
            restart_pos_str = "origin_0"
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
                spawn_point.location.x = spawn_origin[restart_pos_str][0][0]
                spawn_point.location.y = spawn_origin[restart_pos_str][0][1]
                spawn_point.location.z = spawn_origin[restart_pos_str][0][2]
                spawn_point.rotation.yaw = spawn_origin[restart_pos_str][1][0]
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
            spawn_point.location.x = spawn_origin[restart_pos_str][0][0]
            spawn_point.location.y = spawn_origin[restart_pos_str][0][1]
            spawn_point.location.z = spawn_origin[restart_pos_str][0][2]
            spawn_point.rotation.yaw = spawn_origin[restart_pos_str][1][0]
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            if self.player is None:
                print("use random spawn point")
                spawn_point = random.choice(
                    spawn_points) if spawn_points else carla.Transform()
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
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
        ego = self.player
        if ego is None:
            return
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

        # carla cooridnate: x point right, y point downward
        # dynamic passed to planner coordinate: relative to vehicle
        #       x point right, y point forward
        heading_actor = -t_actor.rotation.yaw
        heading_ego = -t_ego.rotation.yaw
        heading = 90 + heading_actor - heading_ego
        while heading < -180:
            heading = heading + 360
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
            elif actor.type_id.startswith('walker.pedestrian'):
                # print("add ped")
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
                abs(rloc_actor.z) <=10
            if not in_vf:
                # print("not in veh frame")
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
        self.world_class = world
        self.world = self.world_class.world
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
        # for spawn_npc and destroy_npc
        self.vehicles_list = []
        self.walkers_list = []
        self.all_id = []
        self.barrels_list = []

    def getDistance(self, p1, p2):
        return math.sqrt((p1.location.x - p2.location.x)**2 + (p1.location.y - p2.location.y)**2)

    def spawn_npc(self, world_class, num_vehicles, num_walkers, autopilot=True):
        world = world_class.world
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)

        try:
            traffic_manager = client.get_trafficmanager(8000)
            traffic_manager.set_global_distance_to_leading_vehicle(2.0)

            #blueprints = world.get_blueprint_library().filter("vehicle.dodge_charger.police")
            blueprints = world.get_blueprint_library().filter('vehicle.*')
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprintsWalkers = world.get_blueprint_library().filter('walker.pedestrian.*')

            if True:
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
                # blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
                # blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
                # blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
                # blueprints = [x for x in blueprints if not x.id.endswith('t2')]

            spawn_points = world.get_map().get_spawn_points()
            number_of_spawn_points = len(spawn_points)

            if num_vehicles < number_of_spawn_points:
                random.shuffle(spawn_points)
            elif num_vehicles > number_of_spawn_points:
                num_vehicles = number_of_spawn_points

            # @todo cannot import these directly.
            SpawnActor = carla.command.SpawnActor
            SetAutopilot = carla.command.SetAutopilot
            SetVehicleLightState = carla.command.SetVehicleLightState
            FutureActor = carla.command.FutureActor

            # --------------
            # Spawn vehicles
            # --------------
            batch = []
            f = open("./sim_params.json",'r')
            npc_points = json.load(f)['npc_points_rel']
            f.close()
            if world_class is None:
                print('there is no player in the map. exit')
                sys.exit()
            ego_transform = world_class.player.get_transform()
            r = ego_transform.get_right_vector()
            # rotate ego coordinate system to carla world coordinate system
            theta = math.atan2(r.y, r.x)
            spawned_vehicles = 0
            for n, transform in enumerate(spawn_points):
                if spawned_vehicles >= num_vehicles:
                    break
                # rel_loc = to_vehframe(ego_vehicle, transform.location)
                if spawned_vehicles<len(npc_points):
                    # transfer coordinate system using rotation mattrix
                    # veh_frame: x point right, y point forward
                    # carla frame: x point right , y point backward
                    x_veh_frame = npc_points[spawned_vehicles][0]
                    y_veh_frame = npc_points[spawned_vehicles][1]
                    transform.location.x = ego_transform.location.x + x_veh_frame*math.cos(theta) + y_veh_frame*math.sin(theta)
                    transform.location.y = ego_transform.location.y - (- x_veh_frame*math.sin(theta) + y_veh_frame*math.cos(theta))
                    transform.location.z = ego_transform.location.z + npc_points[spawned_vehicles][2] + 0.5
                    transform.rotation.yaw = ego_transform.rotation.yaw + math.degrees(npc_points[spawned_vehicles][3])
                blueprint = random.choice(blueprints)
                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)
                if blueprint.has_attribute('driver_id'):
                    driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                    blueprint.set_attribute('driver_id', driver_id)
                blueprint.set_attribute('role_name', 'autopilot')
                spawned_vehicles += 1
                

                # spawn the cars and set their autopilot and light state all together
                batch.append(SpawnActor(blueprint, transform)
                    .then(SetAutopilot(FutureActor, autopilot, traffic_manager.get_port())))

            for response in client.apply_batch_sync(batch, False):
                if response.error:
                    logging.error(response.error)
                else:
                    self.vehicles_list.append(response.actor_id)

            # -------------
            # Spawn Walkers
            # -------------
            # some settings
            percentagePedestriansRunning = 0.0      # how many pedestrians will run
            percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
            # 1. take all the random locations to spawn
            spawn_points = []
            for i in range(num_walkers):
                spawn_point = carla.Transform()
                loc = world.get_random_location_from_navigation()
                if (loc != None):
                    spawn_point.location = loc
                    spawn_points.append(spawn_point)
            # 2. we spawn the walker object
            batch = []
            walker_speed = []
            for spawn_point in spawn_points:
                walker_bp = random.choice(blueprintsWalkers)
                # set as not invincible
                if walker_bp.has_attribute('is_invincible'):
                    walker_bp.set_attribute('is_invincible', 'false')
                # set the max speed
                if walker_bp.has_attribute('speed'):
                    if (random.random() > percentagePedestriansRunning):
                        # walking
                        walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                    else:
                        # running
                        walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
                else:
                    print("Walker has no speed")
                    walker_speed.append(0.0)
                batch.append(SpawnActor(walker_bp, spawn_point))
            results = client.apply_batch_sync(batch, True)
            walker_speed2 = []
            for i in range(len(results)):
                if results[i].error:
                    logging.error(results[i].error)
                else:
                    self.walkers_list.append({"id": results[i].actor_id})
                    walker_speed2.append(walker_speed[i])
            walker_speed = walker_speed2
            # 3. we spawn the walker controller
            batch = []
            walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
            for i in range(len(self.walkers_list)):
                batch.append(SpawnActor(walker_controller_bp, carla.Transform(), self.walkers_list[i]["id"]))
            results = client.apply_batch_sync(batch, True)
            for i in range(len(results)):
                if results[i].error:
                    logging.error(results[i].error)
                else:
                    self.walkers_list[i]["con"] = results[i].actor_id
            # 4. we put altogether the walkers and controllers id to get the objects from their id
            for i in range(len(self.walkers_list)):
                self.all_id.append(self.walkers_list[i]["con"])
                self.all_id.append(self.walkers_list[i]["id"])
            all_actors = world.get_actors(self.all_id)


            # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
            # set how many pedestrians can cross the road
            world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
            for i in range(0, len(self.all_id), 2):
                # start walker
                all_actors[i].start()
                # set walk to random point
                all_actors[i].go_to_location(world.get_random_location_from_navigation())
                # max speed
                all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

            print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(self.vehicles_list), len(self.walkers_list)))

            # 修改限速，括号内参数表示比最高速度低的百分比。如果设置为80，则实际限速为 0.2*v_max
            traffic_manager.global_percentage_speed_difference(50)

        except:
            print('spawn npc failed')

    def destroy_npc(self, world_class):
        world = world_class.world
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)
        print('\ndestroying %d vehicles' % len(self.vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        all_actors = world.get_actors(self.all_id)
        for i in range(0, len(self.all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(self.walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in self.all_id])

        time.sleep(0.5)
        self.vehicles_list = []
        self.walkers_list = []
        self.all_id = []

    def spawn_barrel(self, world_class):
        world = world_class.world
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)

        try:
            blueprints = world.get_blueprint_library().filter("static.prop.barrel")

            spawn_points = world.get_map().get_spawn_points()
            number_of_spawn_points = len(spawn_points)

            # todo cannot import these directly.
            SpawnActor = carla.command.SpawnActor
            FutureActor = carla.command.FutureActor

            # --------------
            # Spawn vehicles
            # --------------
            batch = []
            f = open("./sim_params.json",'r')
            npc_points = json.load(f)['barrel_points_rel']
            f.close()
            if world_class is None:
                print('there is no player in the map. exit')
                sys.exit()
            ego_transform = world_class.player.get_transform()
            r = ego_transform.get_right_vector()
            # rotate ego coordinate system to carla world coordinate system
            theta = math.atan2(r.y, r.x)
            spawned_barrels = 0
            for n, transform in enumerate(spawn_points):
                if spawned_barrels >= 20:
                    break
                # rel_loc = to_vehframe(ego_vehicle, transform.location)
                if spawned_barrels<len(npc_points):
                    # transfer coordinate system using rotation mattrix
                    # veh_frame: x point right, y point forward
                    # carla frame: x point right , y point backward
                    x_veh_frame = npc_points[spawned_barrels][0]
                    y_veh_frame = npc_points[spawned_barrels][1]
                    transform.location.x = ego_transform.location.x + x_veh_frame*math.cos(theta) + y_veh_frame*math.sin(theta)
                    transform.location.y = ego_transform.location.y - (- x_veh_frame*math.sin(theta) + y_veh_frame*math.cos(theta))
                    transform.location.z = ego_transform.location.z + npc_points[spawned_barrels][2]
                    transform.rotation.yaw = ego_transform.rotation.yaw + math.degrees(npc_points[spawned_barrels][3])
                blueprint = random.choice(blueprints)
                spawned_barrels += 1

                batch.append(SpawnActor(blueprint, transform))

            for response in client.apply_batch_sync(batch, False):
                if response.error:
                    logging.error(response.error)
                else:
                    self.barrels_list.append(response.actor_id)
        except:
            print('spawn barrels failed')

    def destroy_barrel(self, world_class):
        world = world_class.world
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)
        print('\ndestroying %d barrels' % len(self.barrels_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in self.barrels_list])
        time.sleep(0.5)

    def destroy_closest_npc(self, world_class):
        world = world_class.world
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)
        ego_location = world_class.player.get_location()
        min_dis = 1000000000
        min_id = -1
        for actor_id in self.vehicles_list:
            actor = world.get_actor(actor_id)
            actor_location = actor.get_location()
            dis = pow(ego_location.x - actor_location.x, 2) + pow(ego_location.y - actor_location.y, 2)
            if (dis < min_dis):
                min_dis = dis
                min_id = actor_id
        if min_id != -1:
            world.get_actor(min_id).destroy()
            self.vehicles_list.remove(min_id)
            print('\ndestroying 1 vehicle')
        print('destroy 1 vehicle failed !')


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
                        world.restart(0)
                        world.player.set_autopilot(True)
                    else:
                        world.restart(0)
                elif event.key > K_0 and event.key <= K_5 and pygame.key.get_mods() & KMOD_CTRL:
                    with open("sim_params.json", 'r') as f:
                        tmp = json.load(f)
                    ego_transform = world.player.get_transform()
                    origin = [[ego_transform.location.x, ego_transform.location.y, ego_transform.location.z + 0.2], [ego_transform.rotation.yaw, 0, 0]]
                    if event.key == K_0:
                        tmp['origin_0'] = origin
                    elif event.key == K_1:
                        tmp['origin_1'] = origin
                    elif event.key == K_2:
                        tmp['origin_2'] = origin
                    elif event.key == K_3:
                        tmp['origin_3'] = origin
                    elif event.key == K_4:
                        tmp['origin_4'] = origin
                    else:
                        tmp['origin_5'] = origin
                    with open("./sim_params.json","w") as f:
                        f.write("{\n")
                        for i,item in enumerate(tmp.items()):
                            if i == len(tmp) - 1:
                                f.write("\""+item[0]+"\""+': '+str(item[1])+'\n')
                                break
                            f.write("\""+item[0]+"\""+': '+str(item[1])+',\n')
                        f.write("\n}")
                elif event.key >= K_0 and event.key <= K_5 and not pygame.key.get_mods() & KMOD_CTRL:
                    if event.key == K_1:
                        world.restart(1)
                    elif event.key == K_2:
                        world.restart(2)
                    elif event.key == K_3:
                        world.restart(3)
                    elif event.key == K_4:
                        world.restart(4)
                    elif event.key == K_5:
                        world.restart(5)
                    else:
                        world.restart(0)
                elif event.key > K_5 and event.key <= K_9:
                    global restart_distance
                    restart_distance = 5
                    world.restart(-1)
                    # restart_distance = int(event.key - K_5) * 5
                    # if self._autopilot_enabled:
                    #     world.player.set_autopilot(False)
                    #     world.restart(-1)
                    #     world.player.set_autopilot(True)
                    # else:
                    #     world.restart(-1)
                elif event.key == K_EQUALS and not pygame.key.get_mods() & KMOD_SHIFT:
                    self.spawn_npc(self.world_class, 20, 0)
                elif event.key == K_EQUALS and pygame.key.get_mods() & KMOD_SHIFT:
                    self.spawn_npc(self.world_class, 20, 0, False)
                elif event.key == K_MINUS:
                    self.destroy_npc(self.world_class)
                elif event.key == K_b and not pygame.key.get_mods() & KMOD_SHIFT:
                    self.spawn_barrel(self.world_class)
                elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                    self.destroy_barrel(self.world_class)
                elif event.key == K_LEFTBRACKET:
                    self.destroy_closest_npc(self.world_class)
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
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
            self._control.speed = 40
        # self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    def _parse_tiev_control(self):
        if not using_zlg_control:
            aimspeed = -_cancontrol.aimspeed if _cancontrol.aimspeed < 0 else _cancontrol.aimspeed
            _pid_controller.tick(aimspeed / 3.6 - _caninfo.carspeed / 360, _cancontrol.aimsteer)
            self._control.steer = _pid_controller.steer
            self._control.brake = _pid_controller.brake
            self._control.throttle = _pid_controller.throttle
            self._control.hand_brake = 0
            self._control.reverse = _cancontrol.aimspeed < 0
        else:
            self._control.steer = _cancontrol.steer
            self._control.brake = _cancontrol.brake
            self._control.throttle = _cancontrol.throttle
            self._control.reverse = _cancontrol.reverse  
            self._control.hand_brake = 0      

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
            'Shortcut Note:',
            'SPACE to on/off TievCtrl',
            'Q to shift reverse gear',
            'BACKSPACE to restart',
            'EQUAL to spawn NPCs',
            'SHIFT+EQUAL for static NPCs',
            'MINUS to destroy all NPCs',
            'SHIFT+0~5 to remember location', 
            '0~5 to go to remembered loc(0~5)',
            '6~9 to trace back 5m']
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            def distance(l): return math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            # for d, vehicle in sorted(vehicles):
            #     if d > 200.0:
            #         break
            #     vehicle_type = get_actor_display_name(vehicle, truncate=22)
            #     self._info_text.append('% 4dm %s' % (d, vehicle_type))

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
        self.fileid = 0


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

            # num_pcd_points = 100    
            # points2pcd(self.points_np[:num_pcd_points*3].reshape(-1, 3), self.fileid)
            # print(self.fileid)
            # self.fileid += 1
            # exit()
            start = time.perf_counter()
            _fusionmap = structFUSIONMAP()
            _fusionmap.cells = []
            _mutex_objectlist.acquire()
            object_list = _objectlist
            _mutex_objectlist.release()
            object_list_pass2cpp = []
            inflation_size = 1 # enlarge bounding box to reduce error
            for i in range(object_list.count):
                obj = object_list.obj[i]
                # make sure two corner point selected is counter-clockwise arranged. 
                object_list_pass2cpp.append([obj.corners.p2.x, obj.corners.p2.y, obj.corners.p1.x, obj.corners.p1.y, \
                    obj.length, obj.width, inflation_size])
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
            controller.destroy_npc(world)
            print('npc destroyed !')


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
        default='1280x720',
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
        help='actor filter (default: "Town03")')
    argparser.add_argument(
        '--zlg',
        action='store_true',
        default=False,
        help='enable autopilot')          
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print('brige running')
    print('using_zlg_control: ', using_zlg_control)
    try:
        game_loop(args)
        global _stop_lcm
        _stop_lcm = True
        _tunnel.unsubscribe(_tunnel_sub)
        

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
