"""CYUL runway geometry, using the simulator threshold coordinates supplied by the user."""

import math

EARTH_RADIUS_M = 6_371_000.0
RUNWAY_THRESHOLDS = {
    "06L": (45.461222, -73.764740),
    "06R": (45.457832, -73.741171),
    "24R": (45.483156, -73.736070),
    "24L": (45.476887, -73.716188),
}
OPPOSITE_RUNWAY = {"06L": "24R", "24R": "06L", "06R": "24L", "24L": "06R"}


def distance_and_bearing(start, end):
    """Return angular distance (radians) and initial true bearing (radians)."""
    lat1, lon1 = map(math.radians, start)
    lat2, lon2 = map(math.radians, end)
    dlon = lon2 - lon1
    a = (math.sin((lat2 - lat1) / 2) ** 2
         + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
    distance = 2 * math.asin(math.sqrt(max(0.0, min(1.0, a))))
    bearing = math.atan2(math.sin(dlon) * math.cos(lat2),
                         math.cos(lat1) * math.sin(lat2)
                         - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    return distance, bearing


def valid_position(latitude, longitude, true_heading):
    return (all(isinstance(v, (int, float)) and math.isfinite(v)
                for v in (latitude, longitude, true_heading))
            and -90 <= latitude <= 90 and -180 <= longitude <= 180
            and (latitude, longitude) != (0, 0))


def detect_runway(latitude, longitude):
    """Select the closest threshold, as in the briefing initialization logic."""
    position = (latitude, longitude)
    return min(RUNWAY_THRESHOLDS,
               key=lambda name: distance_and_bearing(RUNWAY_THRESHOLDS[name], position)[0])


def runway_deviations(runway, latitude, longitude, true_heading):
    """Return signed centerline meters (integer) and heading degrees [-180, 180).

    Left is negative and right is positive looking down the selected runway.
    The centerline extends beyond both thresholds. True bearing is derived from
    the two thresholds, not the magnetic runway designation or cockpit heading.
    """
    start = RUNWAY_THRESHOLDS[runway]
    end = RUNWAY_THRESHOLDS[OPPOSITE_RUNWAY[runway]]
    _, runway_bearing = distance_and_bearing(start, end)
    distance, aircraft_bearing = distance_and_bearing(start, (latitude, longitude))
    cross_track = math.asin(max(-1.0, min(1.0,
        math.sin(distance) * math.sin(aircraft_bearing - runway_bearing)))) * EARTH_RADIUS_M
    heading_deviation = (true_heading - math.degrees(runway_bearing) + 180) % 360 - 180
    return int(round(cross_track)), round(heading_deviation, 2)
