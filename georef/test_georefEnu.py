import unittest
import georefENU as georef
import numpy as np

class TestGeorefEnu(unittest.TestCase):
    def test_georefEnu(self):
        # lgf origin
        lgfOriginLongitude = -68
        lgfOriginLatitude = 48
        lgfOriginLheight = 0
        enuOriginVectorTRF = georef.llh2trf(lgfOriginLongitude, lgfOriginLatitude, lgfOriginLheight)
        trf2enuMatrix = georef.trf2enuMatrix(lgfOriginLongitude, lgfOriginLatitude)

        # Point origin
        lat = -68.000001
        lon = 48.000001
        h = 0
        positionVectorTRF = georef.llh2trf(lon, lat, h)
        positionVectorENU = georef.lgfCoord(positionVectorTRF, enuOriginVectorTRF, trf2enuMatrix)

        # attitude
        rollDegrees = 0
        pitchDegrees = 0
        headingDegrees = 0
        imu2lgfMatrix = georef.imu2enu(rollDegrees, pitchDegrees, headingDegrees)

        # ray vector
        rayX = 0
        rayY = 0
        rayZ = 0
        rayVector = np.array([rayX, rayY, rayZ])

        # boresight
        rollBoresightDegrees = 0
        pitchBoresightDegrees = 0
        headingBoresightDegrees = 0
        boresightMatrix = georef.sensor2imu_boresight(rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees)

        # lever arm
        leverArmX = 0
        leverArmY = 0
        leverArmZ = 0
        leverArm = np.array([leverArmX, leverArmY, leverArmZ])

        xEnu = georef.georefEnu(positionVectorENU, imu2lgfMatrix, rayVector, boresightMatrix, leverArm)

        expectedX = 0 #TODO: establish these
        expectedY = 0 #TODO: establish these
        expectedZ = 0 #TODO: establish these

        print("Test will fail: ")
        print(xEnu)

        self.assertAlmostEqual(expectedX, xEnu[0], delta=1E-3)
        self.assertAlmostEqual(expectedY, xEnu[1], delta=1E-3)
        self.assertAlmostEqual(expectedZ, xEnu[2], delta=1E-3)


if __name__ == '__main__':

    import xmlrunner

    unittest.main(testRunner=xmlrunner.XMLTestRunner(output='test-reports'))

    #unittest.main()