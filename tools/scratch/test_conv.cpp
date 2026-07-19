#include <iostream>
#include <iomanip>
#include "geodetic_converter/geodetic_converter.hpp"

int main() {
    double node_lat = 33.890094;
    double node_lon = 131.002511;
    double robot_lat = 33.88994666666667;
    double robot_lon = 131.002522;

    for (int zone = 1; zone <= 19; ++zone) {
        common::GeodeticConverter conv(zone);
        common::PlaneXY node_xy = conv.geo2xy({node_lat, node_lon, 0.0, 0.0});
        common::PlaneXY robot_xy = conv.geo2xy({robot_lat, robot_lon, 0.0, 0.0});

        double rx = robot_xy.x - node_xy.x;
        double ry = robot_xy.y - node_xy.y;

        std::cout << "Zone " << zone << ": x=" << rx << ", y=" << ry << "\n";
    }

    return 0;
}
