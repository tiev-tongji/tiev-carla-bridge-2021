#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Configure and inspect an instance of CARLA Simulator.

For further details, visit
https://carla.readthedocs.io/en/latest/configuring_the_simulation/
"""

import glob
from math import sqrt
from math import sin
import numpy as np
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import datetime
import re
import socket
import textwrap
import random

def get_ip(host):
    if host in ['localhost', '127.0.0.1']:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect(('10.255.255.255', 1))
            host = sock.getsockname()[0]
        except RuntimeError:
            pass
        finally:
            sock.close()
    return host


def find_weather_presets():
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), x) for x in presets]


def list_options(client):
    maps = [m.replace('/Game/Carla/Maps/', '') for m in client.get_available_maps()]
    indent = 4 * ' '
    def wrap(text):
        return '\n'.join(textwrap.wrap(text, initial_indent=indent, subsequent_indent=indent))
    print('weather presets:\n')
    print(wrap(', '.join(x for _, x in find_weather_presets())) + '.\n')
    print('available maps:\n')
    print(wrap(', '.join(sorted(maps))) + '.\n')


def list_blueprints(world, bp_filter):
    blueprint_library = world.get_blueprint_library()
    blueprints = [bp.id for bp in blueprint_library.filter(bp_filter)]
    print('available blueprints (filter %r):\n' % bp_filter)
    for bp in sorted(blueprints):
        print('    ' + bp)
    print('')


def inspect(args, client):
    address = '%s:%d' % (get_ip(args.host), args.port)

    world = client.get_world()
    elapsed_time = world.get_snapshot().timestamp.elapsed_seconds
    elapsed_time = datetime.timedelta(seconds=int(elapsed_time))

    actors = world.get_actors()
    s = world.get_settings()

    weather = 'Custom'
    current_weather = world.get_weather()
    for preset, name in find_weather_presets():
        if current_weather == preset:
            weather = name

    if s.fixed_delta_seconds is None:
        frame_rate = 'variable'
    else:
        frame_rate = '%.2f ms (%d FPS)' % (
            1000.0 * s.fixed_delta_seconds,
            1.0 / s.fixed_delta_seconds)

    print('-' * 34)
    print('address:% 26s' % address)
    print('version:% 26s\n' % client.get_server_version())
    print('map:        % 22s' % world.get_map().name)
    print('weather:    % 22s\n' % weather)
    print('time:       % 22s\n' % elapsed_time)
    print('frame rate: % 22s' % frame_rate)
    print('rendering:  % 22s' % ('disabled' if s.no_rendering_mode else 'enabled'))
    print('sync mode:  % 22s\n' % ('disabled' if not s.synchronous_mode else 'enabled'))
    print('actors:     % 22d' % len(actors))
    print('  * spectator:% 20d' % len(actors.filter('spectator')))
    print('  * static:   % 20d' % len(actors.filter('static.*')))
    print('  * traffic:  % 20d' % len(actors.filter('traffic.*')))
    print('  * vehicles: % 20d' % len(actors.filter('vehicle.*')))
    print('  * walkers:  % 20d' % len(actors.filter('walker.*')))
    print('-' * 34)

def spawn_barrel_line(world, blueprint, p1, p2):
    v_p1_p2 = p2 - p1
    gap = 0.5
    dis = sqrt(pow(v_p1_p2.x, 2) + pow(v_p1_p2.y, 2))
    num_barrels = int(dis/gap)
    for i in range(num_barrels):
        spawn_point = carla.Transform(p1 + i / (num_barrels-1) * v_p1_p2, carla.Rotation())
        prop = world.try_spawn_actor(blueprint, spawn_point)

def spawn_barrel_sin(world, blueprint, p1, p2):
    v_p1_p2 = p2 - p1
    dis = sqrt(pow(v_p1_p2.x, 2) + pow(v_p1_p2.y, 2))
    vec_norm = np.array([v_p1_p2.y, -v_p1_p2.x, 0])
    vec_direction = np.array([v_p1_p2.x, v_p1_p2.y, 0])
    unit_vec_direction = vec_direction / np.linalg.norm(vec_direction)
    unit_vec_norm = vec_norm / np.linalg.norm(vec_norm)
    origin_p = np.array([p1.x, p1.y, p1.z])
    for i in range(int(dis)):
        sample_point = origin_p + i * unit_vec_direction
        offset = 2 * sin(0.2*i)
        target_left = sample_point + (offset + 2) * unit_vec_norm
        target_right = sample_point + (offset - 2) * unit_vec_norm
        spawn_point = carla.Transform(carla.Location(x = target_left[0], y = target_left[1], z = target_left[2]), carla.Rotation())
        prop = world.try_spawn_actor(blueprint, spawn_point)
        spawn_point = carla.Transform(carla.Location(x = target_right[0], y = target_right[1], z = target_right[2]), carla.Rotation())
        prop = world.try_spawn_actor(blueprint, spawn_point)

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument(
        '-d', '--default',
        action='store_true',
        help='set default settings')
    argparser.add_argument(
        '-m', '--map',
        help='load a new map, use --list to see available maps')
    argparser.add_argument(
        '-r', '--reload-map',
        action='store_true',
        help='reload current map')
    argparser.add_argument(
        '--delta-seconds',
        metavar='S',
        type=float,
        help='set fixed delta seconds, zero for variable frame rate')
    argparser.add_argument(
        '--fps',
        metavar='N',
        type=float,
        help='set fixed FPS, zero for variable FPS (similar to --delta-seconds)')
    argparser.add_argument(
        '--rendering',
        action='store_true',
        help='enable rendering')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        help='disable rendering')
    argparser.add_argument(
        '--no-sync',
        action='store_true',
        help='disable synchronous mode')
    argparser.add_argument(
        '--weather',
        help='set weather preset, use --list to see available presets')
    argparser.add_argument(
        '-i', '--inspect',
        action='store_true',
        help='inspect simulation')
    argparser.add_argument(
        '-l', '--list',
        action='store_true',
        help='list available options')
    argparser.add_argument(
        '-b', '--list-blueprints',
        metavar='FILTER',
        help='list available blueprints matching FILTER (use \'*\' to list them all)')
    argparser.add_argument(
        '-x', '--xodr-path',
        metavar='XODR_FILE_PATH',
        help='load a new map with a minimum physical road representation of the provided OpenDRIVE')

    if len(sys.argv) < 2:
        argparser.print_help()
        return

    args = argparser.parse_args()

    client = carla.Client(args.host, args.port, worker_threads=1)
    client.set_timeout(10.0)

    if args.default:
        args.rendering = True
        args.delta_seconds = 0.0
        args.weather = 'Default'
        args.no_sync = True

    if args.map is not None:
        print('load map %r.' % args.map)
        world = client.load_world(args.map)
        if args.map == 'Town03':
            prop_barrel = random.choice(world.get_blueprint_library().filter("static.prop.barrel"))
            p1 = carla.Location(x = -128.4, y = -10.7, z = 0)
            p2 = carla.Location(x = -128.6, y = 4.8, z = 0)
            spawn_barrel_line(world, prop_barrel, p1, p2)

            p1 = carla.Location(x = 82.9, y = 142.5, z = 0)
            p2 = carla.Location(x = 82.9, y = 120.9, z = 0)
            spawn_barrel_line(world, prop_barrel, p1, p2)

            p1 = carla.Location(x = -96.9, y = -78.4, z = 0)
            p2 = carla.Location(x = -80.1, y = -77.1, z = 0)
            spawn_barrel_line(world, prop_barrel, p1, p2)

            p1 = carla.Location(x = -80.1, y = -77.1, z = 0)
            p2 = carla.Location(x = -80.3, y = -30.8, z = 0)
            spawn_barrel_line(world, prop_barrel, p1, p2)

            p1 = carla.Location(x = 5.9, y = -45, z = 0)
            p2 = carla.Location(x = 7.3, y = -100.9, z = 0)
            spawn_barrel_sin(world, prop_barrel, p1, p2)

            p1 = carla.Location(x = 3.7, y = -44.5, z = 0)
            p2 = carla.Location(x = 1.2, y = -29.8, z = 0)
            spawn_barrel_line(world, prop_barrel, p1, p2)

            p1 = carla.Location(x = 8.5, y = -44.6, z = 0)
            p2 = carla.Location(x = 13.9, y = -46.3, z = 0)
            spawn_barrel_line(world, prop_barrel, p1, p2)

        if args.map == 'Town05':
            # if args.r:
            origin = carla.Location(x=100, y=100, z=0)
            prop_barrel = random.choice(world.get_blueprint_library().filter("static.prop.barrel"))
            for i in range(5,9,1):
                spawn_point = carla.Transform(carla.Location(x=-8,y=i,z=0.0)+origin, carla.Rotation())
                prop = world.try_spawn_actor(prop_barrel, spawn_point)
                prop.set_simulate_physics(True)
            for i in range(-8,-15,-1):
                spawn_point = carla.Transform(carla.Location(x=i,y=9,z=0.0)+origin, carla.Rotation())
                prop = world.try_spawn_actor(prop_barrel, spawn_point)
                prop.set_simulate_physics(True)
            for i in range(9,4,-1):
                spawn_point = carla.Transform(carla.Location(x=-15,y=i,z=0.0)+origin, carla.Rotation())
                prop = world.try_spawn_actor(prop_barrel, spawn_point)
                prop.set_simulate_physics(True)
            for i in range(-8,-15,-1):
                spawn_point = carla.Transform(carla.Location(x=i,y=5,z=0.0)+origin, carla.Rotation())
                prop = world.try_spawn_actor(prop_barrel, spawn_point)
                prop.set_simulate_physics(True)

            for i in range(-7,-3,1):
                spawn_point = carla.Transform(carla.Location(x=10,y=i,z=0.0)+origin, carla.Rotation())
                prop = world.try_spawn_actor(prop_barrel, spawn_point)
                prop.set_simulate_physics(True)
            for i in range(10,3,-1):
                spawn_point = carla.Transform(carla.Location(x=i,y=-3,z=0.0)+origin, carla.Rotation())
                prop = world.try_spawn_actor(prop_barrel, spawn_point)
                prop.set_simulate_physics(True)
            for i in range(-7,-2,1):
                spawn_point = carla.Transform(carla.Location(x=3,y=i,z=0.0)+origin, carla.Rotation())
                prop = world.try_spawn_actor(prop_barrel, spawn_point)
                prop.set_simulate_physics(True)
            for i in range(10,3,-1):
                spawn_point = carla.Transform(carla.Location(x=i,y=-7,z=0.0)+origin, carla.Rotation())
                prop = world.try_spawn_actor(prop_barrel, spawn_point)
                prop.set_simulate_physics(True)

        #prop.set_simulate_physics(True)
    elif args.reload_map:
        print('reload map.')
        world = client.reload_world()

    elif args.xodr_path is not None:
        if os.path.exists(args.xodr_path):
            with open(args.xodr_path) as od_file:
                try:
                    data = od_file.read()
                    print('xodr file found')
                except OSError:
                    print('file could not be readed.')
                    sys.exit()
            print('load opendrive map %r.' % os.path.basename(args.xodr_path))
            vertex_distance = 2.0  # in meters
            max_road_length = 500.0 # in meters
            wall_height = 1.0      # in meters
            extra_width = 0.6      # in meters
            world = client.generate_opendrive_world(
                data, carla.OpendriveGenerationParameters(
                    vertex_distance=vertex_distance,
                    max_road_length=max_road_length,
                    wall_height=wall_height,
                    additional_width=extra_width,
                    smooth_junctions=True,
                    enable_mesh_visibility=True))
        else:
            print('file not found.')

    else:
        world = client.get_world()

    settings = world.get_settings()

    if args.no_rendering:
        print('disable rendering.')
        settings.no_rendering_mode = True
    elif args.rendering:
        print('enable rendering.')
        settings.no_rendering_mode = False

    if args.no_sync:
        print('disable synchronous mode.')
        settings.synchronous_mode = False

    if args.delta_seconds is not None:
        settings.fixed_delta_seconds = args.delta_seconds
    elif args.fps is not None:
        settings.fixed_delta_seconds = (1.0 / args.fps) if args.fps > 0.0 else 0.0

    if args.delta_seconds is not None or args.fps is not None:
        if settings.fixed_delta_seconds > 0.0:
            print('set fixed frame rate %.2f milliseconds (%d FPS)' % (
                1000.0 * settings.fixed_delta_seconds,
                1.0 / settings.fixed_delta_seconds))
        else:
            print('set variable frame rate.')
            settings.fixed_delta_seconds = None

    world.apply_settings(settings)

    if args.weather is not None:
        if not hasattr(carla.WeatherParameters, args.weather):
            print('ERROR: weather preset %r not found.' % args.weather)
        else:
            print('set weather preset %r.' % args.weather)
            world.set_weather(getattr(carla.WeatherParameters, args.weather))

    if args.inspect:
        inspect(args, client)
    if args.list:
        list_options(client)
    if args.list_blueprints:
        list_blueprints(world, args.list_blueprints)


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except RuntimeError as e:
        print(e)
