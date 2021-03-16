import numpy as np


def llh2trf(longitudeDegrees, latitudeDegrees, height, a = 6378137.0, e2 = 0.0066943799901414):
    """
    Determines the ECEF coordinates of the point specified from the longitude, lattitude and height above ellipsoid

    :param longitudeDegrees: longitude in degrees
    :param latitudeDegrees: latitude in degrees
    :param height: height above ellipsoid in meters
    :param a (optional): semi-major axis, defaults to WGS84 value
    :param e2 (optional): first eccentricity squared, defaults to WGS84 value
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
    Determines the rotation matrix associated with a pitch angle.
    Positive angle is a counter clockwise rotation

    :param headingDegrees: heading  angle in degrees
    :return: rotation matrix around body Z axis
    """
    sh = np.sin(np.radians(headingDegrees))
    ch = np.cos(np.radians(headingDegrees))
    return np.array([[ch, -sh, 0], [sh, ch, 0], [0, 0, 1]])


def imu2enu(rollDegrees, pitchDegrees, headingDegrees):
    rollMatrix = rosRollMatrix(rollDegrees)
    pitchMatrix = rosRollMatrix(pitchDegrees)
    headingMatrix = rosRollMatrix(headingDegrees)

    #Validate this convention with unit tests
    return headingMatrix.dot(pitchMatrix.dot(rollMatrix))


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


