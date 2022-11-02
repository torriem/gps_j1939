import math

EARTH_RADIUS = 6378137.0 #metres

"""
    A module to do haversines calculations using longitude and
    latitude coordinates.

    For now it uses ft, though I think we'll want to switch to meters.
"""

def distance(point1, point2):
    distance = math.acos(
                     math.sin(math.radians(point1[1])) * \
                     math.sin(math.radians(point2[1])) + \
                     math.cos(math.radians(point1[1])) * \
                     math.cos(math.radians(point2[1])) * \
                     math.cos(math.radians(point2[0] - point1[0]))) * \
           EARTH_RADIUS

    return abs(distance)

def midpoint(point1, point2):
    lat1 = math.radians(point1[1])
    lon1 = math.radians(point1[0])
    lat2 = math.radians(point2[1])
    lon2 = math.radians(point2[0])

    bx = math.cos(lat2) * math.cos(lon2 - lon1)
    by = math.cos(lat2) * math.sin(lon2 - lon1)

    dlat3 = math.atan2(math.sin(lat1) + math.sin(lat2),
                      math.sqrt((math.cos(lat1)+bx) * \
                                (math.cos(lat1)+bx) + \
                                by * by))
    dlon3 = lon1 + math.atan2(by, math.cos(lat1) + bx)

    center_lat = math.degrees(dlat3)
    center_lon = math.degrees(dlon3)

    return (center_lon, center_lat)

def bearing(point1, point2):
    lat1 = math.radians(point1[1])
    lat2 = math.radians(point2[1])
    delta_lon = math.radians(point2[0] - point1[0])

    # use Haversine formula to calculate bearing between points
    angle = math.atan2( math.sin(delta_lon) * math.cos(lat2),
                        
math.cos(lat1) * math.sin(lat2) - \
                        math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon))

    return (360 + math.degrees(angle)) % 360

def at_distance_bearing (start_pos, heading, distance):
    offset = distance / EARTH_RADIUS
    lat = math.radians(start_pos[1])
    lon = math.radians(start_pos[0])

    lat1sin = math.sin(lat)
    lat1cos = math.cos(lat)
    distcos = math.cos(offset)
    distsin = math.sin(offset)

    newlat = math.asin(lat1sin * distcos +
                       lat1cos * distsin * math.cos(math.radians(heading)))

    newlon = lon + math.atan2(math.sin(math.radians(heading)) * distsin * lat1cos,
                              distcos - lat1sin * math.sin(newlat))

    return (math.degrees(newlon), math.degrees(newlat))


