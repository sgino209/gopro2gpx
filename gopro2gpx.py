#!/usr/bin/env python
#
# 17/02/2019
# Juan M. Casillas <juanm.casillas@gmail.com>
# https://github.com/juanmcasillas/gopro2gpx.git
#
# Released under GNU GENERAL PUBLIC LICENSE v3. (Use at your own risk)
#


import argparse
import array
import os
import platform
import re
import struct
import subprocess
import sys
import time
from collections import namedtuple
from datetime import datetime, timedelta
from config import setup_environment
import fourCC
import gpmf
import gpshelper
from math import atan2, cos, sin, degrees, radians
from geopy import distance

meter_per_second_to_knots = lambda x: x * 1.944

def BuildGPSPoints(data, prev_window=1, skip=False, quiet=False):
    """
    Data comes UNSCALED so we have to do: Data / Scale.
    Do a finite state machine to process the labels.
    GET
     - SCAL     Scale value
     - GPSF     GPS Fix
     - GPSU     GPS Time
     - GPS5     GPS Data
    """

    points = []
    SCAL = fourCC.XYZData(1.0, 1.0, 1.0)
    GPSU = None
    SYST = fourCC.SYSTData(0, 0)
    start_time = None
    start_time_lag_sec = 0

    stats = {
        'ok': 0,
        'badfix': 0,
        'badfixskip': 0,
        'badprecision': 0,
        'badprecisionskip': 0,
        'badaccl': 0,
        'badacclskip': 0,
        'badspeed': 0,
        'badspeedskip': 0,
        'empty' : 0
    }

    SPEED_skip_en = True
    ACCL_skip_en = False
    GPSP_thr = 500  # bad precision
    ACCL_thr = 10   # bad acceleration
    SPEED_thr = 100 # bad speed
    GPSFIX = 0 # no lock.
    lat_prev = 0
    lon_prev = 0
    speed_prev = 0
    for idx,d in enumerate(data):

        if d.fourCC == 'SCAL':
            SCAL = d.data
        
        elif d.fourCC == 'GPSU':
            GPSU = d.data
            if not start_time:
                if GPSFIX == 0:
                    start_time_lag_sec += 1
                else:
                    start_time = gpshelper.UTCTime(datetime.fromtimestamp(time.mktime(GPSU))-timedelta(seconds=start_time_lag_sec))
                    if not quiet:
                        print("start_time change to %s (lag=%dsec)" % (start_time, start_time_lag_sec))
        
        elif d.fourCC == 'GPSP':
            GPSP = d.data
            if GPSP > GPSP_thr:
                stats['badprecision'] += 1
                if skip:
                    if not quiet:
                        print("Warning: Skipping point due bad precision, GPSP=%d>%d" % (GPSP, GPSP_thr))
                    stats['badprecisionskip'] += 1
                    continue
        
        elif d.fourCC == 'ACCL':
            ACCL = d.data
        
        elif d.fourCC == 'GYRO':
            GYRO = d.data
        
        elif d.fourCC == 'GPSF':
            if d.data != GPSFIX:
                if not quiet:
                    print("GPSFIX change to %s [%s]" % (d.data,fourCC.LabelGPSF.xlate[d.data]))
            GPSFIX = d.data
        
        elif d.fourCC == 'GPS5':
            if d.data.lon == d.data.lat == d.data.alt == 0:
                if not quiet:
                    print("Warning: Skipping empty point")
                stats['empty'] += 1
                continue

            if GPSFIX == 0:
                stats['badfix'] += 1
                if skip:
                    if not quiet:
                        print("Warning: Skipping point due GPSFIX==0")
                    stats['badfixskip'] += 1
                    continue

            data = [ float(x) / float(y) for x,y in zip( d.data._asdict().values() ,list(SCAL) ) ]
            gpsdata = fourCC.GPSData._make(data)
            speed_kn = meter_per_second_to_knots(gpsdata.speed)
            acceleration = speed_kn - speed_prev

            if SPEED_skip_en and (abs(speed_kn) > SPEED_thr):
                stats['badspeed'] += 1
                if skip:
                    if not quiet:
                        print("Warning: Skipping point due bad speed, abs(SPEED)=%d>%d" % (abs(speed_kn), SPEED_thr))
                    stats['badspeedskip'] += 1
                    continue
            
            if ACCL_skip_en and (abs(acceleration) > ACCL_thr):
                stats['badaccl'] += 1
                if skip:
                    if not quiet:
                        print("Warning: Skipping point due bad acceleration, abs(ACCL)=%d>%d" % (abs(acceleration), ACCL_thr))
                    stats['badacclskip'] += 1
                    continue

            # Bearing calculation:
            direction_y = sin(radians(gpsdata.lon) - radians(lon_prev)) * cos(radians(gpsdata.lat))
            direction_x = cos(radians(lat_prev)) * sin(radians(gpsdata.lat)) - sin(radians(lat_prev)) * cos(radians(gpsdata.lat)) * cos(radians(gpsdata.lon) - radians(lon_prev))
            direction = (degrees(atan2(direction_y, direction_x)) + 360) % 360

            # Distance calculation:
            dist_2d_geopy = distance.distance((lat_prev, lon_prev), (gpsdata.lat, gpsdata.lon)).m if lat_prev*lon_prev > 0 else 0

            if idx % prev_window == 0:
                lat_prev = gpsdata.lat
                lon_prev = gpsdata.lon
            speed_prev = speed_kn

            p = gpshelper.GPSPoint(gpsdata.lat, 
                                   gpsdata.lon, 
                                   gpsdata.alt, 
                                   datetime.fromtimestamp(time.mktime(GPSU)),
                                   speed_kn,
                                   dist_2d_geopy,
                                   direction,
                                   GPSP,
                                   GPSFIX,
                                   ACCL,
                                   GYRO)
            points.append(p)
            stats['ok'] += 1

        elif d.fourCC == 'SYST':
            data = [ float(x) / float(y) for x,y in zip( d.data._asdict().values() ,list(SCAL) ) ]
            if data[0] != 0 and data[1] != 0:
                SYST = fourCC.SYSTData._make(data)

        elif d.fourCC == 'GPRI':
            # KARMA GPRI info

            if d.data.lon == d.data.lat == d.data.alt == 0:
                if not quiet:
                    print("Warning: Skipping empty point")
                stats['empty'] += 1
                continue

            if GPSFIX == 0:
                stats['badfix'] += 1
                if skip:
                    if not quiet:
                        print("Warning: Skipping point due GPSFIX==0")
                    stats['badfixskip'] += 1
                    continue
                    
            data = [ float(x) / float(y) for x,y in zip( d.data._asdict().values() ,list(SCAL) ) ]
            gpsdata = fourCC.KARMAGPSData._make(data)
            
            if SYST.seconds != 0 and SYST.miliseconds != 0:
                p = gpshelper.GPSPoint(gpsdata.lat, 
                                       gpsdata.lon, 
                                       gpsdata.alt, 
                                       datetime.fromtimestamp(SYST.miliseconds),
                                       gpsdata.speed) 
                points.append(p)
                stats['ok'] += 1
                        
    print("-- stats -----------------")
    total_points =0
    for i in stats.keys():
        total_points += stats[i]
    print("- Ok (valid points):\t%5d" % stats['ok'])
    print("- GPSFIX=0 (bad)\t%5d (skipped: %d)" % (stats['badfix'], stats['badfixskip']))
    print("- GPSP>%d (bad):\t%5d (skipped: %d)" % (GPSP_thr, stats['badprecision'], stats['badprecisionskip']))
    print("- abs(ACCL)>%d (bad):\t%5d (skipped: %d)" % (ACCL_thr, stats['badaccl'], stats['badacclskip']))
    print("- abs(SPEED)>%d (bad):\t%5d (skipped: %d)" % (SPEED_thr, stats['badspeed'], stats['badspeedskip']))
    print("- Empty (No data):\t%5d" % stats['empty'])
    print("Total points:\t\t%5d" % total_points)
    print("--------------------------")
    
    return points, start_time

def parseArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="count")
    parser.add_argument("-b", "--binary", help="read data from bin file", action="store_true")
    parser.add_argument("-s", "--skip", help="Skip bad points (GPSFIX=0)", action="store_true", default=False)
    parser.add_argument("-q", "--quiet", help="Quiet mode", action="store_true", default=False)
    parser.add_argument("-a", "--speed_dir_window", help="average window size for calculating speed and direction", default=1)
    parser.add_argument("file", help="Video file or binary metadata dump")
    parser.add_argument("outputfile", help="output file. builds KML and GPX")
    args = parser.parse_args()

    return args        

def main():
    args = parseArgs()
    config = setup_environment(args)
    parser = gpmf.Parser(config)

    if not args.binary:
        data = parser.readFromMP4()
    else:
        data = parser.readFromBinary()

    # build some funky tracks from camera GPS

    points, start_time = BuildGPSPoints(data, prev_window=int(args.speed_dir_window), skip=args.skip, quiet=args.quiet)

    if len(points) == 0:
        print("Can't create file. No GPS info in %s. Exitting" % args.file)
        sys.exit(0)

    kml = gpshelper.generate_KML(points)
    with open("%s.kml" % args.outputfile , "w+") as fd:
        fd.write(kml)

    gpx = gpshelper.generate_GPX(points, start_time, trk_name="gopro7-track")
    with open("%s.gpx" % args.outputfile , "w+") as fd:
        fd.write(gpx)

if __name__ == "__main__":
    main()
