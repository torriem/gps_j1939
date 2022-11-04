#/usr/bin/python3

import haversines
import math

# tractor: 120 inches above ground, 53 inches lateral between antennas, 56 inches forward of axle

INCHES = 0.0254 # inches per meter
GPS_HEIGHT = 120 * INCHES # distance above axle to primary antenna
GPS_RIGHT = 53 / 2.0 * INCHES # lateral distance from right antenna to center of axle
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
        side_heading = (heading - 90) % 360 #90 degrees to left of vehicle heading

    lat_offset = math.sin(math.radians(roll)) * GPS_HEIGHT
    #print (side_heading, lat_offset / INCHES)
    
    alt_offset = math.cos(math.radians(roll)) * GPS_HEIGHT
    #print (alt_offset / INCHES)


    if GPS_RIGHT:
        additional_lat_offset = math.cos(math.radians(roll)) * GPS_RIGHT
        additional_alt_offset = math.sin(math.radians(roll)) * GPS_RIGHT
    else:
        additional_lat_offset = 0
        additional_alt_offset = 0

    #print ("Heading %f, offset %f, additional_offset %f" % (side_heading, lat_offset, additional_lat_offset))
    #print ("lat offset total: %f" % (lat_offset + additional_lat_offset))
    #print ("altitude %f, offset %f, additional_offset %f" % (gps_pos[2], alt_offset, additional_alt_offset))
    new_pos = haversines.at_distance_bearing( (gps_pos[0], gps_pos[1]), side_heading, lat_offset + additional_lat_offset)
    new_alt = gps_pos[2] - alt_offset + additional_alt_offset


 
    # translate forward or back to axle from antenna, ignoring pitch of vehicle
    if GPS_FORWARD:
        new_pos = haversines.at_distance_bearing( new_pos, heading, -GPS_FORWARD)

    #print (new_pos)
    #print (new_pos1)
    #print ()

    return ( new_pos[0], new_pos[1], new_alt )

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

    class KFilter:
        def __init__(self, P, process, var):
            self.P = P
            self.varProcess = process
            self.var = var

            self.Xe = 0

        def filter(self, reading):
            Pc = self.P + self.varProcess  #error estimate
            G = Pc / (Pc + self.var) # gain
            self.P = (1 - G) * Pc
            Xp = self.Xe
            Zp = Xp
            self.Xe = (G * (reading - Zp)) + Xp

            return self.Xe

    class KFilter1:
        def __init__(self, meas_error, est_error, q):
            self.err_measure = meas_error
            self.err_estimate = est_error
            self.q = q
            self.cur_est = 0
            self.last_est = 0
            self.gain = 0

        def filter( self, mea):
            self.gain = self.err_estimate / (self.err_estimate + self.err_measure)
            self.cur_est = self.last_est + self.gain * (mea - self.last_est)
            self.err_estimate = (1.0 - self.gain) * self.err_estimate + abs(self.last_est - self.cur_est) * self.q
            self.last_est = self.cur_est
            return self.cur_est
    

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
        #serialport = sys.argv[1]
        inputfile = open(sys.argv[1],'r')

    else:
        serialport = '/dev/ttyUSB3'
        serial = serial.Serial(serialport, baudrate = 115200)

        thread = threading.Thread(target=serial_readline, args = (serial, ),).start()

    roll = 0
    last_heading = 0

    #yaw_filter = KFilter(1.0, 0.0003, 0.1)
    yaw_filter = KFilter1(0.1, 1.0, 0.001)
    roll_filter = KFilter1(0.1, 1.0, 0.001)
    
    #while True:
    for line in inputfile:
        #if queue.empty(): continue
        #line = queue.get(True, 1)

        line = line.strip()
        #log_serial_data(seriallog, line)
        try:
            msg = pynmea2.parse(line)

            # we need both a STI030 and STI036 that line up

            if isinstance(msg, pynmea2.types.proprietary.sti.STI030):
                if not (position.time == msg.time and position.date == msg.date):
                    # start a new position
                    position.attitude = None
                    position.time = msg.time
                    position.date = msg.date

                position.pos = msg

                if position.pos and position.attitude:
                    print ("New position.")

            if isinstance(msg, pynmea2.types.proprietary.sti.STI036):
                if not (position.time == msg.time and position.date == msg.date):
                    # start a new position
                    position.pos = None
                    position.time = msg.time
                    position.date = msg.date

                position.attitude = msg

                # vehicle heading is 90 degrees less than this reported heading
                position.attitude.heading90 = position.attitude.heading
                position.attitude.heading = (position.attitude.heading90 + 90) % 360

                #print (position.attitude.pitch)

                #position.attitude.pitch = roll_filter.filter(position.attitude.pitch)

                # TODO: filter roll just like AOG does
                ##Pc = position.P + Position.varProcess
                ##G = Pc / (Pc + Position.varRoll)
                ##position.P = (1 - G) * Pc
                ##Xp = position.XeRoll
                ##Zp = Xp
                ##position.XeRoll = (G * (position.attitude.pitch - Zp)) + Xp
                ##position.attitude.roll = position.XeRoll

                #print ('vehicle heading %f, roll %f, orig altitude %f' % (position.attitude.heading, position.attitude.roll, position.pos.altitude))

                if position.pos and position.attitude:
                    #print (position.pos.longitude, position.pos.latitude)
                    
                    heading_delta = position.attitude.heading - last_heading
                    if heading_delta >180: heading_delta -= 360
                    if heading_delta <=-180: heading_delta += 360
                    yaw_rate = heading_delta / 0.2

                    #yaw_rate = yaw_filter.filter(yaw_rate)

                    last_heading = position.attitude.heading

                    compensated_pos = virtual_position( (position.pos.longitude, position.pos.latitude, position.pos.altitude), position.attitude.roll, position.attitude.heading, position.attitude.heading90)
                    #print (position.attitude.heading, position.attitude.roll, compensated_pos)
                    print ( "%.7f, %.7f, %f, hdg %f, %f, %f" % ( *compensated_pos, position.attitude.heading, position.attitude.pitch, yaw_rate))
                    #log_gps( compensated_pos, position, logfile)

            #print(repr(msg))
        except pynmea2.ParseError as e:
            #print('Parse error: {}'.format(e))
            continue

    #print(virtual_position((-110,49,792),-0.2,180))

