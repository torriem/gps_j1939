#/usr/bin/python3

import haversines
import math

# tractor: 120 inches above ground, 53 inches lateral between antennas, 56 inches forward of axle

INCHES = 0.0254 # inches per meter
GPS_HEIGHT = 120 * INCHES # distance above axle to primary antenna
GPS_LEFT = 53 / 2.0 * INCHES # lateral distance from primary antenna to center of axle
GPS_FORWARD = 0 * INCHES # distance in front of axle

last_gps_pos = None

"""
    Test facility for dual GPS, 5 Hz

    - Use autotrac and SF1 to drive a straight course.

    - read in custom NMEA setences
    - translate to axle using heading, tilt angle
    - plot UTM position?

    - continuously read IMU tilt data, maybe at 20 hz or something
    - smooth filter?
    - record IMU data for analysis
    - translate GPS to axle using heading, IMU tilt angle
    - compare to dual GPS axle position

    questions:
    - is there correlation between dual GPS roll and IMU
    - how much latency is there between GPS and IMU that correlates
    - is GPS with IMU more accurate than dual GPS?

    Notes
    - if gps 1 is on the port side of vehicle, heading points toward
      gps 2.  Pitch (roll) is positive to the left, negative to the right

"""

def virtual_position (gps_pos, roll, heading, side_heading = None):
    #if abs(roll) < 0.01: return gps_pos

    if side_heading is None:
        side_heading = (heading + 90) % 360 #90 degrees to right of vehicle heading

    lat_offset = math.sin(math.radians(roll)) * GPS_HEIGHT
    #print (side_heading, lat_offset / INCHES)
    
    alt_offset = math.cos(math.radians(roll)) * GPS_HEIGHT
    #print (alt_offset / INCHES)


    if GPS_LEFT:
        additional_offset = math.cos(math.radians(roll)) * GPS_LEFT
        print ("Heading %f, offset %f, additional_offset %f" % (side_heading, lat_offset, additional_offset))

    new_pos1 = haversines.at_distance_bearing( (gps_pos[0], gps_pos[1]), side_heading, lat_offset + additional_offset)

    new_pos = haversines.at_distance_bearing( (gps_pos[0], gps_pos[1]), side_heading, lat_offset)


    # translate forward or back to axle from antenna, ignoring pitch of vehicle
    if GPS_FORWARD:
        new_pos = haversines.at_distance_bearing( new_pos, heading, -GPS_FORWARD)

    if GPS_LEFT:
        new_pos = haversines.at_distance_bearing( new_pos, side_heading, GPS_LEFT)

    print (haversines.distance( (gps_pos[0], gps_pos[1]), new_pos))
    print (lat_offset+additional_offset)
    
    #print (new_pos)
    #print (new_pos1)
    #print ()

    return ( new_pos[0], new_pos[1], gps_pos[2] - alt_offset )

def log_gps( gps_pos, position, logfile):
    global last_gps_pos

    if last_gps_pos:
        dist = haversines.distance(last_gps_pos, gps_pos)
        if dist < 0.1:
            return

    # Either we're just starting out or we've covered
    # at least half a meter since the last logged
    # position, so let's log a new position.

    print ("logging.")
    logfile.write('%.7f, %.7f, %.2f, %.7f, %.7f, %.2f, %.2f, %.2f\n' % (gps_pos[0], gps_pos[1], gps_pos[2], position.pos.longitude, position.pos.latitude, position.pos.altitude, position.attitude.pitch, position.attitude.heading))
    logfile.flush()
    last_gps_pos = gps_pos

def log_serial_data(to_file, line):
    to_file.write(line);
    to_file.write('\n');
        

if __name__ == "__main__":
    import pynmea2
    import sys
    import queue
    import threading
    import serial

    queue = queue.Queue(1000)
    quit_threads = False

    class Position:
        varProcess = 0.0003
        varRoll = 0.1
        yawrate_varProcess = 0.0003
        yawrate_varR = 0.1

        def __init__(self):
            self.pos = None
            self.attitude = None
            self.time = None
            self.date = None
            self.new_pos = False

            #self.Pc = 0
            #self.G = 0
            #self.Xp = 0
            #self.Zp = 0
            self.XeRoll = 0
            self.P = 1.0

    def serial_readline(s):
        while True and not quit_threads:
            line = s.readline().decode('utf-8')
            queue.put(line)

    position = Position()

    logfile = open ("gps.log", "a", encoding = 'utf-8')
    seriallog = open ("serial.log", "a", encoding = 'utf-8')

    #file = open(sys.argv[1], encoding='utf-8')

    if len(sys.argv) > 1:
        serialport = sys.argv[1]
    else:
        serialport = '/dev/ttyUSB3'

    serial = serial.Serial(serialport, baudrate = 115200)

    thread = threading.Thread(target=serial_readline, args = (serial, ),).start()

    roll = 0

    while True:
        if queue.empty(): continue
        line = queue.get(True, 1)

        log_serial_data(seriallog, line)
        try:
            msg = pynmea2.parse(line)

            # we need both a GGA and a STI032 that line up

            if isinstance(msg, pynmea2.types.GGA):
                if not (position.time == msg.time and position.date == msg.date):
                    # start a new position
                    position.attitude = None
                    position.time = msg.time
                    position.date = msg.date

                position.pos = msg

                if position.pos and position.attitude:
                    print ("New position.")

            # 035 or 032? AOG uses 032 but docs are fuzzy
            if isinstance(msg, pynmea2.types.proprietary.sti.STI032):
                if not (position.time == msg.time and position.date == msg.date):
                    # start a new position
                    position.pos = None
                    position.time = msg.time
                    position.date = msg.date

                position.attitude = msg

                # vehicle heading is 90 degrees less than this reported heading
                if position.attitude.bearing: #bearing is always right-hand moving base towards left-hand rover
                    position.attitude.heading = (position.attitude.bearing + 90) % 360
                    position.attitude.pitch = math.atan(position.attitude.elevation / position.attitude.dist)

                    # TODO: filter roll just like AOG does
                    Pc = position.P + Position.varProcess
                    G = Pc / (Pc + varRoll)
                    position.P = (1 - G) * Pc
                    Xp = position.XeRoll
                    Zp = Xp
                    position.XeRoll = (G * (position.attitude.pitch - Zp)) + Xp
                    position.attitude.pitch = position.XeRoll

                    # might need to negate the pitch
                    print ('vehicle heading %f, pitch %f, height diff %f' % (position.attitude.heading, position.attitude.pitch, position.attitude.elevation))

                else:
                    position.attitude.heading = 0
                    position.attitude.pitch = 0

                if position.pos and position.attitude:
                    #print (position.pos.longitude, position.pos.latitude)

                    compensated_pos = virtual_position( (position.pos.longitude, position.pos.latitude, position.pos.altitude), position.attitude.pitch, position.attitude.heading, position.attitude.bearing)
                    print (position.attitude.heading, position.attitude.pitch, compensated_pos)
                    log_gps( compensated_pos, position, logfile)

            #print(repr(msg))
        except pynmea2.ParseError as e:
            #print('Parse error: {}'.format(e))
            continue

    #print(virtual_position((-110,49,792),-0.2,180))

