import unittest
import georefENU as georef
import numpy as np

class TestGeorefEnu(unittest.TestCase):


    def test_geo2cart_atEquator(self):
        lon = 0
        lat = 0
        h = 0

        X_trf = georef.geo2cart(lon, lat, h)

        expectedX = georef.a # semi major axis
        expectedY = 0
        expectedZ = 0

        self.assertAlmostEqual(expectedX, X_trf[0], delta=1E-6) # 1 micrometer
        self.assertAlmostEqual(expectedY, X_trf[1], delta=1E-6) # 1 micrometer
        self.assertAlmostEqual(expectedZ, X_trf[2], delta=1E-6) # 1 micrometer


    def test_geo2cart_atPoles(self):
        lon = 0
        lat = 90
        h = 0

        X_trf = georef.geo2cart(lon, lat, h)

        expectedX = 0
        expectedY = 0
        expectedZ = georef.b # semi-minor axis

        self.assertAlmostEqual(expectedX, X_trf[0], delta=1E-6) # 1 micrometer
        self.assertAlmostEqual(expectedY, X_trf[1], delta=1E-6) # 1 micrometer
        self.assertAlmostEqual(expectedZ, X_trf[2], delta=1E-6) # 1 micrometer


    def test_geo2cart_and_cart2geo(self):
        lon = 31.45
        lat = -68.79
        h = 12.93

        x, y, z = georef.geo2cart(lon, lat, h)

        testLon, testLat, testHeight = georef.cart2geo(x, y, z)

        degree_eps = 1E-12 # less than 1 micrometer at equator
        height_eps = 1E-9 # 1 nanometer

        self.assertAlmostEqual(lon, testLon, delta=degree_eps)
        self.assertAlmostEqual(lat, testLat, delta=degree_eps)
        self.assertAlmostEqual(h, testHeight, delta=height_eps)


    def test_trf2enuMatrixAt0lon0lat(self):
        trf2enu = georef.trf2enuMatrix(0, 0)
        # At lon=0 and lat=0, the X axis of the ECEF is aligned with the up axis of ENU
        np.testing.assert_array_almost_equal(trf2enu[:,0],
                                             np.array([0,0,1]),
                                             decimal=6, # 1 micrometer
                                             err_msg="At lon=0 and lat=0 the X axis of ECEF should be aligned with the UP axis of ENU")

        # At lon=0 and lat=0, the Y axis of the ECEF is aligned with the east axis of ENU
        np.testing.assert_array_almost_equal(trf2enu[:, 1],
                                             np.array([1, 0, 0]),
                                             decimal=6, # 1 micrometer
                                             err_msg="At lon=0 and lat=0 the Y axis of ECEF should be aligned with the EAST axis of ENU")

        # At lon=0 and lat=0, the Z axis of the ECEF is aligned with the north axis of ENU
        np.testing.assert_array_almost_equal(trf2enu[:, 2],
                                             np.array([0, 1, 0]),
                                             decimal=6, # 1 micrometer
                                             err_msg="At lon=0 and lat=0 the Z axis of ECEF should be aligned with the NORTH axis of ENU")


    def test_trf2enuMatrixAt90lon0lat(self):
        trf2enu = georef.trf2enuMatrix(90, 0)
        # At lon=90 and lat=0, the X axis of the ECEF is aligned with the negative of east axis of ENU
        np.testing.assert_array_almost_equal(trf2enu[:,0],
                                             np.array([-1,0,0]),
                                             decimal=6, # 1 micrometer
                                             err_msg="At lon=90 and lat=0 the X axis of ECEF should be aligned with the negative of EAST axis of ENU")

        # At lon=90 and lat=0, the Y axis of the ECEF is aligned with the UP axis of ENU
        np.testing.assert_array_almost_equal(trf2enu[:, 1],
                                             np.array([0, 0, 1]),
                                             decimal=6, # 1 micrometer
                                             err_msg="At lon=90 and lat=0 the Y axis of ECEF should be aligned with the UP axis of ENU")

        # At lon=90 and lat=0, the Z axis of the ECEF is aligned with the north axis of ENU
        np.testing.assert_array_almost_equal(trf2enu[:, 2],
                                             np.array([0, 1, 0]),
                                             decimal=6, # 1 micrometer
                                             err_msg="At lon=90 and lat=0 the Z axis of ECEF should be aligned with the NORTH axis of ENU")


    def test_lgfCoordinates_1mEast(self):
        lonLgfOrigin = 0
        latLgfOrigin = 0
        heightLgfOrigin = 0

        trf2enu = georef.trf2enuMatrix(lonLgfOrigin, latLgfOrigin)

        lgfOriginCoordinatedInEcef = georef.geo2cart(lonLgfOrigin, latLgfOrigin, heightLgfOrigin)

        # Create a point 1 m east of this origin and express it in geographic coordinates
        point1mEastCoordinatedInEcef = lgfOriginCoordinatedInEcef + np.array([0,1,0]) #at lon = 0 and lat = 0, East in ENU is aligned with the Y axis of ECEF
        testPositionInGeographic = georef.cart2geo(point1mEastCoordinatedInEcef[0], point1mEastCoordinatedInEcef[1], point1mEastCoordinatedInEcef[2])

        # Verify that this test position is [1, 0, 0]
        testPositionInEcef = georef.geo2cart(testPositionInGeographic[0], testPositionInGeographic[1], testPositionInGeographic[2])
        testPositionInEnu = georef.lgfCoord(testPositionInEcef, lgfOriginCoordinatedInEcef, trf2enu)
        np.testing.assert_array_almost_equal(testPositionInEnu,
                                             np.array([1, 0, 0]),
                                             decimal=6, # 1 micrometer
                                             err_msg="Wrong ENU coordinates for a point 1m east of origin")


    def test_lgfCoordinates_1mNorth(self):
        lonLgfOrigin = 0
        latLgfOrigin = 0
        heightLgfOrigin = 0

        trf2enu = georef.trf2enuMatrix(lonLgfOrigin, latLgfOrigin)

        lgfOriginCoordinatedInEcef = georef.geo2cart(lonLgfOrigin, latLgfOrigin, heightLgfOrigin)

        # Create a point 1 m north of this origin and express it in geographic coordinates
        point1mNorthCoordinatedInEcef = lgfOriginCoordinatedInEcef + np.array([0, 0, 1]) #at lon = 0 and lat = 0, North in ENU is aligned with Z axis of ECEF
        testPositionInGeographic = georef.cart2geo(point1mNorthCoordinatedInEcef[0], point1mNorthCoordinatedInEcef[1], point1mNorthCoordinatedInEcef[2])

        # Verify that this test position is [0, 1, 0] in ENU frame
        testPositionInEcef = georef.geo2cart(testPositionInGeographic[0], testPositionInGeographic[1], testPositionInGeographic[2])
        testPositionInEnu = georef.lgfCoord(testPositionInEcef, lgfOriginCoordinatedInEcef, trf2enu)
        np.testing.assert_array_almost_equal(testPositionInEnu,
                                             np.array([0, 1, 0]),
                                             decimal=6, # 1 micrometer
                                             err_msg="Wrong ENU coordinates for a point 1m north of origin")


    def test_lgfCoordinates_GeneralPosition(self):
        lonLgfOrigin = -68
        latLgfOrigin = 48
        heightLgfOrigin = -28

        trf2enu = georef.trf2enuMatrix(lonLgfOrigin, latLgfOrigin)

        lgfOriginCoordinatedInEcef = georef.geo2cart(lonLgfOrigin, latLgfOrigin, heightLgfOrigin)

        # Create a general position from this ENU frame and express is in geographic coordinates
        expectedEnuPositionVectorCoordinatedInEnu = np.array([106.32, 53.72, -8.94])
        expectedEnuPositionVectorCoordinatedInEcef = trf2enu.transpose().dot(expectedEnuPositionVectorCoordinatedInEnu)
        generalPositionCoordinatedInEcef = lgfOriginCoordinatedInEcef + expectedEnuPositionVectorCoordinatedInEcef
        tesLon, testLat, testHeight = georef.cart2geo(generalPositionCoordinatedInEcef[0], generalPositionCoordinatedInEcef[1], generalPositionCoordinatedInEcef[2])

        # Verify that this test position corresponds to expected ENU position
        testPositionInEcef = georef.geo2cart(tesLon, testLat, testHeight)

        # this is the function call that is tested in this unit test
        testPositionInEnu = georef.lgfCoord(testPositionInEcef, lgfOriginCoordinatedInEcef, trf2enu)
        np.testing.assert_array_almost_equal(testPositionInEnu,
                                             expectedEnuPositionVectorCoordinatedInEnu,
                                             decimal=6, # 1 micrometer
                                             err_msg="Wrong ENU coordinates for the point [106.32, 53.72, -8.94]")


    def test_imuMatrixConsitency(self):
        roll = -2.01
        pitch = 1.37
        heading = 15.13

        imu2enu_1 = georef.imu2enuDecomposed(roll, pitch, heading)
        imu2enu_2 = georef.imu2enu(roll, pitch, heading)

        np.testing.assert_array_almost_equal(imu2enu_1,
                                             imu2enu_2,
                                             decimal=16,
                                             err_msg="imu 2 enu matrix direct computation differs from product of elementary rotation matrices")


    def test_georefEnu(self):
        # lgf origin
        lgfOriginLongitude = 0
        lgfOriginLatitude = 0
        lgfOriginLheight = 0
        enuOriginVectorTRF = georef.geo2cart(lgfOriginLongitude, lgfOriginLatitude, lgfOriginLheight)

        #test origin of ENU frame
        expectedX = georef.a  # semi major axis
        expectedY = 0
        expectedZ = 0

        self.assertAlmostEqual(expectedX, enuOriginVectorTRF[0], delta=1E-6)  # 1 micrometer
        self.assertAlmostEqual(expectedY, enuOriginVectorTRF[1], delta=1E-6)  # 1 micrometer
        self.assertAlmostEqual(expectedZ, enuOriginVectorTRF[2], delta=1E-6)  # 1 micrometer
        #done test origin of ENU frame



        trf2enuMatrix = georef.trf2enuMatrix(lgfOriginLongitude, lgfOriginLatitude)

        # test trf2enuMatrix
        # At lon=0 and lat=0, the X axis of the ECEF is aligned with the up axis of ENU
        np.testing.assert_array_almost_equal(trf2enuMatrix[:, 0],
                                             np.array([0, 0, 1]),
                                             decimal=6,  # 1 micrometer
                                             err_msg="At lon=0 and lat=0 the X axis of ECEF should be aligned with the UP axis of ENU")

        # At lon=0 and lat=0, the Y axis of the ECEF is aligned with the east axis of ENU
        np.testing.assert_array_almost_equal(trf2enuMatrix[:, 1],
                                             np.array([1, 0, 0]),
                                             decimal=6,  # 1 micrometer
                                             err_msg="At lon=0 and lat=0 the Y axis of ECEF should be aligned with the EAST axis of ENU")

        # At lon=0 and lat=0, the Z axis of the ECEF is aligned with the north axis of ENU
        np.testing.assert_array_almost_equal(trf2enuMatrix[:, 2],
                                             np.array([0, 1, 0]),
                                             decimal=6,  # 1 micrometer
                                             err_msg="At lon=0 and lat=0 the Z axis of ECEF should be aligned with the NORTH axis of ENU")
        # done test trf2enuMatrix

        # antenna location
        lat = 0
        lon = 0
        h = 0
        positionVectorTRF = georef.geo2cart(lon, lat, h)

        # test positionVectorTRF of antenna
        expectedAntXTrf = georef.a  # semi major axis
        expectedAntYTrf = 0
        expectedAntZTrf = 0

        self.assertAlmostEqual(expectedAntXTrf, positionVectorTRF[0], delta=1E-6)  # 1 micrometer
        self.assertAlmostEqual(expectedAntYTrf, positionVectorTRF[1], delta=1E-6)  # 1 micrometer
        self.assertAlmostEqual(expectedAntZTrf, positionVectorTRF[2], delta=1E-6)  # 1 micrometer
        # done test positionVectorTRF of antenna

        positionVectorENU = georef.lgfCoord(positionVectorTRF, enuOriginVectorTRF, trf2enuMatrix)

        # test positionVectorENU of antenna, should be 0 since we're at the origin
        expectedAntXEnu = 0
        expectedAntYEnu = 0
        expectedAntZEnu = 0

        self.assertAlmostEqual(expectedAntXEnu, positionVectorENU[0], delta=1E-6)  # 1 micrometer
        self.assertAlmostEqual(expectedAntYEnu, positionVectorENU[1], delta=1E-6)  # 1 micrometer
        self.assertAlmostEqual(expectedAntZEnu, positionVectorENU[2], delta=1E-6)  # 1 micrometer
        # done test positionVectorENU of antenna

        # attitude
        rollDegrees = 0
        pitchDegrees = 0
        headingDegrees = 0
        imu2lgfMatrix = georef.imu2enu(rollDegrees, pitchDegrees, headingDegrees)

        # test imu2enu matrix
        np.testing.assert_array_almost_equal(imu2lgfMatrix,
                                             np.identity(3),
                                             decimal=16,
                                             err_msg="imu2lgfMatrix should be identity with null roll pitch and heading")
        # done test imu2enu matrix

        # ray vector, this is in the body frame
        rayX = 1
        rayY = 0
        rayZ = 0
        rayVector = np.array([rayX, rayY, rayZ])

        # test ray vector
        expectedrayVector = np.array([1, 0, 0])

        np.testing.assert_array_almost_equal(rayVector,
                                             expectedrayVector,
                                             decimal=16,
                                             err_msg="ray vector should be [1, 0, 0] which is the unit forward vector in body frame")
        # done test ray vector

        # boresight
        rollBoresightDegrees = 0
        pitchBoresightDegrees = 0
        headingBoresightDegrees = 0
        boresightMatrix = georef.sensor2imu_boresight(rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees)

        # test boresightMatrix matrix
        np.testing.assert_array_almost_equal(boresightMatrix,
                                             np.identity(3),
                                             decimal=16,
                                             err_msg="boresightMatrix should be identity with null misalignment")
        # done test boresightMatrix matrix

        # lever arm
        leverArmX = 0
        leverArmY = 0
        leverArmZ = 0
        leverArm = np.array([leverArmX, leverArmY, leverArmZ])

        # test lever arm
        expectedLeverArmVector = np.array([0, 0, 0])

        np.testing.assert_array_almost_equal(leverArm,
                                             expectedLeverArmVector,
                                             decimal=16,
                                             err_msg="Lever arm should be [0, 0, 0] for this test")
        # done test lever arm



        xEnu = georef.georefEnu(positionVectorENU, imu2lgfMatrix, rayVector, boresightMatrix, leverArm)

        expectedEnuVector = np.array([1, 0, 0])

        # test georef
        np.testing.assert_array_almost_equal(xEnu,
                                             expectedEnuVector,
                                             decimal=16,
                                             err_msg="georeferenced position should be [1, 0, 0] for this test")
        # done test georef

    def test_georefEnu(self):
        # lgf origin
        lgfOriginLongitude = 0
        lgfOriginLatitude = 0
        lgfOriginLheight = 0
        enuOriginVectorTRF = georef.geo2cart(lgfOriginLongitude, lgfOriginLatitude, lgfOriginLheight)

        #test origin of ENU frame
        expectedX = georef.a  # semi major axis
        expectedY = 0
        expectedZ = 0

        self.assertAlmostEqual(expectedX, enuOriginVectorTRF[0], delta=1E-6)  # 1 micrometer
        self.assertAlmostEqual(expectedY, enuOriginVectorTRF[1], delta=1E-6)  # 1 micrometer
        self.assertAlmostEqual(expectedZ, enuOriginVectorTRF[2], delta=1E-6)  # 1 micrometer
        #done test origin of ENU frame



        trf2enuMatrix = georef.trf2enuMatrix(lgfOriginLongitude, lgfOriginLatitude)

        # test trf2enuMatrix
        # At lon=0 and lat=0, the X axis of the ECEF is aligned with the up axis of ENU
        np.testing.assert_array_almost_equal(trf2enuMatrix[:, 0],
                                             np.array([0, 0, 1]),
                                             decimal=6,  # 1 micrometer
                                             err_msg="At lon=0 and lat=0 the X axis of ECEF should be aligned with the UP axis of ENU")

        # At lon=0 and lat=0, the Y axis of the ECEF is aligned with the east axis of ENU
        np.testing.assert_array_almost_equal(trf2enuMatrix[:, 1],
                                             np.array([1, 0, 0]),
                                             decimal=6,  # 1 micrometer
                                             err_msg="At lon=0 and lat=0 the Y axis of ECEF should be aligned with the EAST axis of ENU")

        # At lon=0 and lat=0, the Z axis of the ECEF is aligned with the north axis of ENU
        np.testing.assert_array_almost_equal(trf2enuMatrix[:, 2],
                                             np.array([0, 1, 0]),
                                             decimal=6,  # 1 micrometer
                                             err_msg="At lon=0 and lat=0 the Z axis of ECEF should be aligned with the NORTH axis of ENU")
        # done test trf2enuMatrix

        # antenna location
        lat = 0
        lon = 0
        h = 0
        positionVectorTRF = georef.geo2cart(lon, lat, h)

        # test positionVectorTRF of antenna
        expectedAntXTrf = georef.a  # semi major axis
        expectedAntYTrf = 0
        expectedAntZTrf = 0

        self.assertAlmostEqual(expectedAntXTrf, positionVectorTRF[0], delta=1E-6)  # 1 micrometer
        self.assertAlmostEqual(expectedAntYTrf, positionVectorTRF[1], delta=1E-6)  # 1 micrometer
        self.assertAlmostEqual(expectedAntZTrf, positionVectorTRF[2], delta=1E-6)  # 1 micrometer
        # done test positionVectorTRF of antenna

        positionVectorENU = georef.lgfCoord(positionVectorTRF, enuOriginVectorTRF, trf2enuMatrix)

        # test positionVectorENU of antenna, should be 0 since we're at the origin
        expectedAntXEnu = 0
        expectedAntYEnu = 0
        expectedAntZEnu = 0

        self.assertAlmostEqual(expectedAntXEnu, positionVectorENU[0], delta=1E-6)  # 1 micrometer
        self.assertAlmostEqual(expectedAntYEnu, positionVectorENU[1], delta=1E-6)  # 1 micrometer
        self.assertAlmostEqual(expectedAntZEnu, positionVectorENU[2], delta=1E-6)  # 1 micrometer
        # done test positionVectorENU of antenna

        # attitude
        rollDegrees = 0
        pitchDegrees = 0
        headingDegrees = 90
        imu2lgfMatrix = georef.imu2enu(rollDegrees, pitchDegrees, headingDegrees)

        imu2lgfMatrixProductOfElementaryRotations = georef.imu2enuDecomposed(rollDegrees, pitchDegrees, headingDegrees)

        # test imu2enu matrix
        np.testing.assert_array_almost_equal(imu2lgfMatrix,
                                             imu2lgfMatrixProductOfElementaryRotations,
                                             decimal=16,
                                             err_msg="imu2lgfMatrix should be equal to product of elementary rotations")
        # done test imu2enu matrix

        # ray vector, this is in the body frame
        rayX = 1
        rayY = 0
        rayZ = 0
        rayVector = np.array([rayX, rayY, rayZ])

        # test ray vector
        expectedrayVector = np.array([1, 0, 0])

        np.testing.assert_array_almost_equal(rayVector,
                                             expectedrayVector,
                                             decimal=16,
                                             err_msg="ray vector should be [1, 0, 0] which is the unit forward vector in body frame")
        # done test ray vector

        # boresight
        rollBoresightDegrees = 0
        pitchBoresightDegrees = 0
        headingBoresightDegrees = 0
        boresightMatrix = georef.sensor2imu_boresight(rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees)

        # test boresightMatrix matrix
        np.testing.assert_array_almost_equal(boresightMatrix,
                                             np.identity(3),
                                             decimal=16,
                                             err_msg="boresightMatrix should be identity with null misalignment")
        # done test boresightMatrix matrix

        # lever arm
        leverArmX = 0
        leverArmY = 0
        leverArmZ = 0
        leverArm = np.array([leverArmX, leverArmY, leverArmZ])

        # test lever arm
        expectedLeverArmVector = np.array([0, 0, 0])

        np.testing.assert_array_almost_equal(leverArm,
                                             expectedLeverArmVector,
                                             decimal=16,
                                             err_msg="Lever arm should be [0, 0, 0] for this test")
        # done test lever arm



        xEnu = georef.georefEnu(positionVectorENU, imu2lgfMatrix, rayVector, boresightMatrix, leverArm)

        expectedEnuVector = np.array([0, 1, 0])

        # test georef
        np.testing.assert_array_almost_equal(xEnu,
                                             expectedEnuVector,
                                             decimal=16,
                                             err_msg="georeferenced position should be [0, 1, 0] for this test")
        # done test georef

if __name__ == '__main__':

    #import xmlrunner

    #unittest.main(testRunner=xmlrunner.XMLTestRunner(output='test-reports'))

    unittest.main()