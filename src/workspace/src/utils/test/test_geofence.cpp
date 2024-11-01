#include <iostream>
#include <vector>
#include "../Geofence.hpp"


void test_is_within() {
	const std::string wkt = "MULTIPOLYGON (((30 20, 45 40, 10 40, 30 20)),((15 5, 40 10, 10 20, 5 10, 15 5)))";
	auto gf = Geofence(wkt);
	std::vector<Geofence::GeofencePoint> points = {
		// In the first Polygon
		{31, 22},
		{40, 35},
		{30, 35},
		{18, 35},
		// In the second polygon
		{18, 15},
		{20, 10},
		{25, 14},
		{39, 10},
	};
	for (Geofence::GeofencePoint p : points) {
		if (not gf.PointInGeofence(p.lon, p.lat))
			throw std::runtime_error("Test fails");
		std::cout << "Passing: " << p.lon << " " << p.lat << std::endl;
	}
}

void test_is_outside() {
	const std::string wkt = "MULTIPOLYGON (((30 20, 45 40, 10 40, 30 20)),((15 5, 40 10, 10 20, 5 10, 15 5)))";
	auto gf = Geofence(wkt);
	std::vector<Geofence::GeofencePoint> points = {
		// Big values away from both polygons
		{180, 90},
		{180, -90},
		{0, 90},
		{0, -0},
		{179, 89},
		//Edgeing but still outside first polygon
		{45, 41},
		{25, 25},
		{20, 30},
		{25, 25},
		//Edgeing but still outside second polygon
		{15, 4},
		{23, 6},
		{30, 14},
		{6, 14},

	};
	for (Geofence::GeofencePoint p : points) {
		if (gf.PointInGeofence(p.lon, p.lat))
			throw std::runtime_error("Test fails");
		std::cout << "Passing: " << p.lon << " " << p.lat << std::endl;
	}
}

void test_valid_two_polygon_multipolygon() {
	const std::string wkt = "MULTIPOLYGON (((30 20, 45 40, 10 40, 30 20)),((15 5, 40 10, 10 20, 5 10, 15 5)))";
	auto gf = Geofence(wkt);
	if (gf.mp.size() != 2)
		throw std::runtime_error("There should be two polygons in the multipolygon!");

	// Check first polygon matches
	const std::vector<Geofence::GeofencePoint> & polygon1 = gf.mp[0];
	if (polygon1.size() != 4)
		throw std::runtime_error("There should be 4 points in the polygon");
	const std::vector<Geofence::GeofencePoint> points1 = {{30, 20}, {45, 40}, {10, 40}, {30, 20}};
	for (const int i : {0,1,2,3}) {
		const Geofence::GeofencePoint p1 = polygon1[i];
		const Geofence::GeofencePoint p2 = points1[i];
		std::cout << "Passing: " << p1.lon << " " << p1.lat << std::endl;
		if (p1.lon != p2.lon or p1.lat != p2.lat)
			throw std::runtime_error("The points are wrong!");
	}

	// Check second polygon matches
	const std::vector<Geofence::GeofencePoint> & polygon2 = gf.mp[1];
	if (polygon2.size() != 5)
		throw std::runtime_error("There should be 5 points in the polygon");
	const std::vector<Geofence::GeofencePoint> points2 = {{15, 5}, {40, 10}, {10, 20}, {5, 10}, {15, 5}};
	for (const int i : {0,1,2,3,4}) {
		const Geofence::GeofencePoint p1 = polygon2[i];
		const Geofence::GeofencePoint p2 = points2[i];
		std::cout << "Passing: " << p1.lon << " " << p1.lat << std::endl;
		if (p1.lon != p2.lon or p1.lat != p2.lat)
			throw std::runtime_error("The points are wrong!");
	}
}

void test_valid_one_polygon_multipolygon() {
	const std::string wkt = "MULTIPOLYGON (((99 89, 40 30, 23 56, 80 30, 99 89)))";
	auto gf = Geofence(wkt);
	if (gf.mp.size() != 1)
		throw std::runtime_error("There should be one polygon in the multipolygon!");

	// Check first polygon matches
	const std::vector<Geofence::GeofencePoint> & polygon = gf.mp[0];
	if (polygon.size() != 5)
		throw std::runtime_error("There should be 5 points in the polygon");
	std::vector<Geofence::GeofencePoint> points = {{99, 89}, {40, 30}, {23, 56}, {80, 30},{99, 89}};
	for (int i : {0,1,2,3,4}) {
		Geofence::GeofencePoint p1 = polygon[i];
		Geofence::GeofencePoint p2 = points[i];
		std::cout << "Passing: " << p1.lon << " " << p1.lat << std::endl;
		if (p1.lon != p2.lon or p1.lat != p2.lat)
			throw std::runtime_error("The points are wrong!");
	}
}

void test_valid_one_polygon_one_point_multipolygon() {
	const std::string wkt = "MULTIPOLYGON (((99 89, 99 89)))";
	auto gf = Geofence(wkt);
	if (gf.mp.size() != 1)
		throw std::runtime_error("There should be one polygon in the multipolygon!");

	// Check first polygon matches
	const std::vector<Geofence::GeofencePoint> & polygon = gf.mp[0];
	if (polygon.size() != 2)
		throw std::runtime_error("There should be 2 points in the polygon");
	std::vector<Geofence::GeofencePoint> points = {{99, 89}, {99, 89}};
	for (int i : {0,1}) {
		Geofence::GeofencePoint p1 = polygon[i];
		Geofence::GeofencePoint p2 = points[i];
		std::cout << "Passing: " << p1.lon << " " << p1.lat << std::endl;
		if (p1.lon != p2.lon or p1.lat != p2.lat)
			throw std::runtime_error("The points are wrong!");
	}
}

void test_invalid_one_polygon_multipolygon() {
	const std::string wkt = "MULTIPOLYGON (((99 80,";
	bool threw = false;
	try {
		auto gf = Geofence(wkt);
	} catch (std::runtime_error & e) {
		threw = true;
		std::cout << "Passing: " << wkt << " was correctly found bad: " << e.what() << std::endl;
	}
	if (!threw)
		throw std::runtime_error("This wkt should not be allowed");
}

void test_invalid_one_polygon_multipolygon_one_point(){
	const std::string wkt = "MULTIPOLYGON (((99 100)))";
	bool threw = false;
	try {
		auto gf = Geofence(wkt);
	} catch (std::runtime_error & e) {
		threw = true;
		std::cout << "Passing: " << wkt << " was correctly found bad: " << e.what() << std::endl;
	}
	if (!threw)
		throw std::runtime_error("This wkt should not be allowed");
}

void test_invalid_two_polygon_multipolygon() {
	const std::string wkt = "MULTIPOLYGON (((30 20, 45 40, 10 40, 30 21)),((15 5, 40 10, 10 20, 5 10, 15 5)))";
	bool threw = false;
	try {
		auto gf = Geofence(wkt);
	} catch (std::runtime_error & e) {
		threw = true;
		std::cout << "Passing: " << wkt << " was correctly found bad: " << e.what() << std::endl;
	}
	if (!threw)
		throw std::runtime_error("This wkt should not be allowed");
}

void test_invalid_point_in_two_polygon_multipolygon() {
	const std::string wkt = "MULTIPOLYGON (((30 20, 45 40, I0 40, 30 20)),((15 5, 40 10, 10 20, 5 10, 15 5)))";
	bool threw = false;
	try {
		auto gf = Geofence(wkt);
	} catch (std::runtime_error & e) {
		threw = true;
		std::cout << "Passing: " << wkt << " was correctly found bad: " << e.what() << std::endl;
	}
	if (!threw)
		throw std::runtime_error("This wkt should not be allowed");
}

void test_valid_polygon_with_decimals() {
	const std::string wkt = "MULTIPOLYGON (((30.53 20.35643, 45.46224 40.464, 10.5345 40.3, 30.53 20.35643)),((15.1111 5.222, 40 10.56, 10.99 20, 5.1 10.4543355, 15.1111 5.222)))";
	auto gf = Geofence(wkt);
	std::cout << "Passing: " << wkt << std::endl;
}

void test_valid_polygon_with_many_decimals() {
	const std::string wkt = "MULTIPOLYGON (((15.1111 5.222366543, 40 10.56, 10.9956654345 20, 5.1 10.4543355, 15.1111 5.222366543)))";
    const auto gf = Geofence(wkt);
    if (gf.mp[0][2].lon == 10.9956654345 && gf.mp[0][2].lon != 10.9956654346 && gf.mp[0][2].lon != 10.9956654344) {
		std::cout << "Passing: " << wkt << std::endl;
	} else {
		throw std::runtime_error("We require many decimals and good precision!");
	}
}


void test_parse_wkt() {
	// Test some valid multipolygon work
	test_valid_two_polygon_multipolygon();
	test_valid_one_polygon_multipolygon();
	test_valid_one_polygon_one_point_multipolygon();

	// Test some invalid multipolygon are rejected
	test_invalid_one_polygon_multipolygon();
	test_invalid_one_polygon_multipolygon_one_point();
	test_invalid_two_polygon_multipolygon();
	test_invalid_point_in_two_polygon_multipolygon();

	// Test some decimals as well
	test_valid_polygon_with_decimals();
	test_valid_polygon_with_many_decimals();
}

int main() {
	test_is_within();
	test_is_outside();
	test_parse_wkt();
	return 0;
}
