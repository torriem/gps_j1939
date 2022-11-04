#import pynmea2
#import gps
#import datetime
#import math
import time
import sys
import pynmea2
import re
import datetime
import os

""" This script tries to replay a data file that was recorded from 
    a GPS receiver outputting NMEA strings.  It attempts to replicate
    the original timing of the messages by looking at the timestamps
    when applicable and delaying output the same amount it would have
    been delayed originally.  For example, NMEA messages usually occur
    at 1 Hz or 5 Hz.  Also this script will ignore any bad data and
    pass it on through, just like it was recorded originally, which 
    should help with testing the robustness of the NMEA parsing
    routine on our AgOpenGPS project.
"""
johndeere = re.compile(r'\x02.*\x03')
nmea_sentence = re.compile('\$.*$')


def timed_read(read_from):
    last_time = 0
    last_nmea_time = None
    date = None
    last_timestamp = None
    for line in read_from:
        m = nmea_sentence.match(line)
        if m: #we found what looks like an nmea sentence
            # try to parse it.
            try:
                nmea = None
                nmea = pynmea2.parse(m.group(0))
                try:
                    date = nmea.datestamp
                except AttributeError:
                    pass

                try:
                    timestamp = nmea.timestamp
                except AttributeError:
                    timestamp = None
                    # if there's no timestamp in the message
                    # we'll pass it on immediately
                    pass
                
                if timestamp:
                    # if there is a timestamp, we'll see if we need to wait
                    # before sending it on to replicate the original timing
                    # of the nmea data stream
                    if date:
                        timestamp = datetime.datetime.combine(date,timestamp)
                    else:
                        timestamp = datetime.datetime.combine(datetime.date.today(),timestamp)
                    
                    if last_nmea_time:
                        wait_time = (timestamp - last_nmea_time).total_seconds()
                        while (time.time() - last_time) < wait_time:
                            time.sleep(0.001)

                    last_nmea_time = timestamp
               
            except pynmea2.nmea.ParseError:
                pass
            
            last_time = time.time()
            yield (line.strip(),nmea) #simply yield the line immediately

def read_three_lines(read_from):
    while 1:
            one = read_from.readline().strip()
            two = read_from.readline().strip()
            three = read_from.readline().strip()
            four = read_from.readline().strip()

            if one and two and three:
                yield (one, two, three, four)
            else:
                return


if __name__== '__main__':
    nmeafile=open(sys.argv[1],"r")
    #nmeafile=open("moving2.txt","r")

    for line in timed_read(nmeafile):
        #try:
        #    sys.stderr.write(str(line[1].true_course))
        #    sys.stderr.write("\n")
        #except AttributeError:
        #    pass

        sys.stdout.write(line[0] + chr(13) + chr(10))
        sys.stdout.flush()
        """
        if type(line[1]) is pynmea2.RMC:
            lat = line[1].latitude
            lon = line[1].longitude
            print utm.from_latlon(lat,lon)
        """

    sys.stderr.write("\n")


