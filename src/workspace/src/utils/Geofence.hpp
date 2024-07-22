#ifndef GEOFENCE_HPP
#define GEOFENCE_HPP

#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>

class Geofence {
public:
    struct GeofencePoint {
        double lon, lat;
    };

    explicit Geofence(const std::string & wkt){
        this->geofence = wkt;
        this->mp = ParseWKT(this->geofence);
    }

    bool PointInGeofence(double lon, double lat) const{
        return PointInMultipolygon({lon, lat}, mp);
    }
    std::string geofence;
    std::vector<std::vector<GeofencePoint>> mp;
    std::string world_wkt = "MULTIPOLYGON(((180 90, -180 90, -180 -90, 180 -90, 180 90)))";
private:
    static bool PointInMultipolygon(const GeofencePoint & point, const std::vector<std::vector<GeofencePoint>> & mp){
        return std::any_of(mp.begin(), mp.end(), [&](const std::vector<GeofencePoint> & polygon) {
            return PointInPolygon(point, polygon);
        });
    }

    // Lightly modified version of https://www.geeksforgeeks.org/point-in-polygon-in-cpp/
    static bool PointInPolygon(const GeofencePoint & point, const std::vector<GeofencePoint> & polygon) {
        const int n = static_cast<int>(polygon.size());
        int count = 0;
        for (int i = 0; i < n; i++) {
            GeofencePoint p1 = polygon[i];
            GeofencePoint p2 = polygon[(i + 1) % n];
            if ((point.lon > std::min(p1.lon, p2.lon))
                && (point.lon <= std::max(p1.lon, p2.lon))
                && (point.lat <= std::max(p1.lat, p2.lat))) {
                const double intersect = (point.lon - p1.lon) * (p2.lat - p1.lat) / (p2.lon - p1.lon) + p1.lat;
                if (p1.lat == p2.lat || point.lat <= intersect)
                    count++;
            }
        }
        const bool isInside = count % 2 == 1;
        return isInside;
    }

    // !! This parser is not fully implemented and only correctly parses a subset of possible WKT !!
    static std::vector<std::vector<GeofencePoint>> ParseWKT(const std::string & wkt){
        std::vector<std::vector<GeofencePoint>> mp;
        const std::string::size_type startPos = wkt.find("(((");
        const std::string::size_type endPos = wkt.find(")))");
        if (startPos == std::string::npos or endPos == std::string::npos){
            std::string msg = "Malformed wkt missing '(((' or ')))'" + wkt;
            throw std::runtime_error(msg);
        }
        std::string polygons = wkt.substr(startPos + 3, endPos - startPos - 3);
        const std::string delimiter = ")),((";
        size_t pos;
        while ((pos = polygons.find(delimiter)) != std::string::npos) {
            const std::string polygon = polygons.substr(0, pos);
            mp.push_back(GetPolygonFromPolygonStr(polygon));
            polygons.erase(0, pos + delimiter.length());
        }
        // The last implicit split remaining in the polygons string
        if (!polygons.empty())
            mp.push_back(GetPolygonFromPolygonStr(polygons));
        if (mp.empty())
            throw std::runtime_error("Couldn't find any polygons in the multipolygon: " + wkt);
        return mp;
    }

    static std::vector<GeofencePoint> GetPolygonFromPolygonStr(std::string polygonStr){
        std::vector<GeofencePoint> polygon {};
        const std::string delimiter = ",";
        size_t pos;
        while ((pos = polygonStr.find(delimiter)) != std::string::npos) {
            const std::string pointStr = polygonStr.substr(0, pos);
            polygon.push_back(GetPointFromPointStr(pointStr));
            polygonStr.erase(0, pos + delimiter.length());
        }
        if (!polygonStr.empty())
            // Get the last implicit point from the polygon string
            polygon.push_back(GetPointFromPointStr(polygonStr));
        if (polygon.empty() or polygon.size() == 1)
            throw std::runtime_error("Polygon must have at least two points: " + polygonStr);
        GeofencePoint f = polygon.front();
        GeofencePoint b = polygon.back();
        if (f.lon != b.lon or f.lat != b.lat)
            throw std::runtime_error("The first and last point in polygon should be the same: " + polygonStr);
        return polygon;
    }

    static GeofencePoint GetPointFromPointStr(const std::string & pointStr){
        double lon, lat;
        try {
            auto ss = std::stringstream(pointStr);
            std::string lon_str, lat_str;
            ss >> lon_str >> lat_str;
            // std::stod is called directly since it will produce exception on bad input, direct stringstream to double
            // will not produce the exception that we can catch as a malformed point
            lon = std::stod(lon_str);
            lat = std::stod(lat_str);
        } catch (...) {
            throw std::runtime_error("Malformed point: " + pointStr);
        }
        if (lon < -180.0 || lon > 180.0)
            throw std::runtime_error("Longitude values should be between -180.0 and 180.0 : " + pointStr);
        if (lat < -90.0 || lat > 90.0)
            throw std::runtime_error("Latitude values should be between -90.0 and 90.0 : " + pointStr);
        return {lon, lat};
    }
};
#endif