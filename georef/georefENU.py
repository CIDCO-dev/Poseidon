import numpy as np

# ellipsoid parameters default to WGS84 values
a = 6378137.0             # semi major axis
f = 1/298.257223563       # flattening
b = a * (1 - f)           # semi minor acis
e2 = f*(2-f)              # first eccentricity squared
epsilon = e2 / (1.0 - e2) # second eccentricity squared

def modifyEllipsoidParameters(semiMajorAxis, flattening):
    global a, f, b, e2, epsilon
    a = semiMajorAxis
    f = flattening
    b = a*(1-f)
    e2 = f*(2-f)
    epsilon = e2 / (1.0 - e2)

def cart2geo(x, y, z):
    """
    Determines the geographic position associated with ECEF vectors using Bowring (1985) algorithm

    :param xyz: ECEF vectors
    :return: longitude, latitude and heigh of corresponding ECEF vectors
    """

    p = np.sqrt(x*x + y*y)
    r = np.sqrt(x * x + y * y + z*z)

    tanu = (1 - f) * (z / p) * (1 + epsilon * b / r);
    tan2u = tanu * tanu

    cos2u = 1.0 / (1.0 + tan2u)
    cosu = np.sqrt(cos2u)
    cos3u = cos2u * cosu

    sinu = tanu * cosu
    sin2u = 1.0 - cos2u
    sin3u = sin2u * sinu

    tanlat = (z + epsilon * b * sin3u) / (p - e2 * a * cos3u)
    tan2lat = tanlat * tanlat
    cos2lat = 1.0 / (1.0 + tan2lat)
    sin2lat = 1.0 - cos2lat

    coslat = np.sqrt(cos2lat)
    sinlat = tanlat * coslat

    longitude = np.arctan2(y, x)
    latitude = np.arctan(tanlat)
    height = p * coslat + z * sinlat - a * np.sqrt(1.0 - e2 * sin2lat)

    llh = (np.rad2deg(longitude),np.rad2deg(latitude),height)
    return llh

def geo2cart(longitudeDegrees, latitudeDegrees, height):
    """
    Determines the ECEF coordinates of the point specified from the longitude, lattitude and height above ellipsoid

    :param longitudeDegrees: longitude in degrees
    :param latitudeDegrees: latitude in degrees
    :param height: height above ellipsoid in meters
    :return: vector of position in ECEF
    """

    slon = np.sin(np.radians(longitudeDegrees))
    clon = np.cos(np.radians(longitudeDegrees))

    slat = np.sin(np.radians(latitudeDegrees))
    clat = np.cos(np.radians(latitudeDegrees))

    N = a / (np.sqrt(1 - e2 * slat * slat))

    x_trf = (N + height) * clat * clon
    y_trf = (N + height) * clat * slon
    z_trf = (N * (1 - e2) + height) * slat

    return np.array([x_trf, y_trf, z_trf])


def trf2enuMatrix(longitudeDegrees, latitudeDegrees):
    """
    Determines the rotation matrix from the ECEF to ENU frame.

    see https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates
    equation 6

    :param longitudeDegrees: longitude of the ENU frame's origin in degrees
    :param latitudeDegrees: latitude of the ENU frame's origin in degrees
    :return: Rotation matrix from ECEF to ENU
    """

    slon = np.sin(np.radians(longitudeDegrees))
    clon = np.cos(np.radians(longitudeDegrees))

    slat = np.sin(np.radians(latitudeDegrees))
    clat = np.cos(np.radians(latitudeDegrees))

    return np.array([[-slon, clon, 0], [-clon*slat, -slon*slat, clat], [clon*clat, slon*clat, slat]])

def lgfCoord(point, lgfOrigin, trf2lgfMatrix):
    """
    Determines the coordinates of a point in a Local Geodetic Frame.

    :param point: Coordinates of the point of interest expressed in ECEF
    :param lgfOrigin: Coordinates of the LGF origin expressed in ECEF
    :param lgf2trfMatrix: Rotation matrix from
    :return: Position vector of the point coordinated in the LGF
    """
    return trf2lgfMatrix.dot(point - lgfOrigin)


def rosRollMatrix(rollDegrees):
    """
    Determines the rotation matrix associated with a roll angle.
    Positive angle is a counter clockwise rotation

    :param rollDegrees: roll angle in degrees
    :return: rotation matrix around body X axis
    """
    sr = np.sin(np.radians(rollDegrees))
    cr = np.cos(np.radians(rollDegrees))
    return np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def rosPitchMatrix(pitchDegrees):
    """
    Determines the rotation matrix associated with a pitch angle.
    Positive angle is a counter clockwise rotation

    :param pitchDegrees: pitch angle in degrees
    :return: rotation matrix around body Y axis
    """
    sp = np.sin(np.radians(pitchDegrees))
    cp = np.cos(np.radians(pitchDegrees))
    return np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])


def rosHeadingMatrix(headingDegrees):
    """
    Determines the rotation matrix associated with a heading angle.
    Positive angle is a counter clockwise rotation

    :param headingDegrees: heading  angle in degrees
    :return: rotation matrix around body Z axis
    """
    sh = np.sin(np.radians(headingDegrees))
    ch = np.cos(np.radians(headingDegrees))
    return np.array([[ch, -sh, 0], [sh, ch, 0], [0, 0, 1]])


def imu2enuDecomposed(rollDegrees, pitchDegrees, headingDegrees):
    rollMatrix = rosRollMatrix(rollDegrees)
    pitchMatrix = rosPitchMatrix(pitchDegrees)
    headingMatrix = rosHeadingMatrix(headingDegrees)

    #Validate this convention with unit tests
    return headingMatrix.dot(pitchMatrix.dot(rollMatrix))


def imu2enu(rollDegrees, pitchDegrees, headingDegrees):
    sr = np.sin(np.radians(rollDegrees))
    cr = np.cos(np.radians(rollDegrees))

    sp = np.sin(np.radians(pitchDegrees))
    cp = np.cos(np.radians(pitchDegrees))

    sh = np.sin(np.radians(headingDegrees))
    ch = np.cos(np.radians(headingDegrees))

    return np.array([[ch*cp, ch*sp*sr-sh*cr, ch*sp*cr+sh*sr], [sh*cp, sh*sp*sr+ch*cr, sh*sp*cr-ch*sr], [-sp, cp*sr, cp*cr]])


def sensor2imu_boresight(deltaRollDegrees, deltaPitchDegrees, deltaHeadingDegrees):
    return imu2enu(deltaRollDegrees, deltaPitchDegrees, deltaHeadingDegrees)


def georefEnu(positionVector, imu2lgfMatrix, rayVector, boresightMatrix, leverArmVector):
    """

    :param positionVector:
    :param imu2lgfMatrix:
    :param rayVector:
    :param boresightMatrix:
    :param leverArmVector:
    :return:
    """
    return positionVector + imu2lgfMatrix.dot(boresightMatrix.dot(rayVector) + leverArmVector)


