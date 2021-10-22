#include <iostream>
#include <memory>
#include "integration_test_helper.h"
#include "mavsdk.h"
#include "plugins/telemetry/telemetry.h"
#include "plugins/geofence/geofence.h"

using namespace mavsdk;

static Geofence::Point add_point(double latitude_deg, double longitude_deg);

TEST_F(SitlTest, GeofenceInclusion)
{
    Mavsdk mavsdk;

    ConnectionResult ret = mavsdk.add_udp_connection();
    ASSERT_EQ(ret, ConnectionResult::Success);

    // Wait for system to connect via heartbeat.
    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto system = mavsdk.systems().at(0);
    ASSERT_TRUE(system->has_autopilot());
    auto telemetry = std::make_shared<Telemetry>(system);
    auto geofence = std::make_shared<Geofence>(system);

    while (!telemetry->health_all_ok()) {
        LogInfo() << "waiting for system to be ready";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    LogInfo() << "System ready, let's start";

    // Get the home position so the geofence points are set with respect
    // to the home position instead of being hardcoded.
    auto home = telemetry->home();

    std::vector<Geofence::Point> points;
    points.push_back(add_point(home.latitude_deg + 0.00154170, home.longitude_deg - 0.00264216));
    points.push_back(add_point(home.latitude_deg - 0.00078588, home.longitude_deg - 0.00399400));
    points.push_back(add_point(home.latitude_deg - 0.00148309, home.longitude_deg - 0.00033547));
    points.push_back(add_point(home.latitude_deg + 0.00205002, home.longitude_deg + 0.00175310));

    std::vector<Geofence::Polygon> polygons;
    Geofence::Polygon new_polygon{};
    new_polygon.fence_type = Geofence::Polygon::FenceType::Inclusion;
    new_polygon.points = points;

    polygons.push_back(new_polygon);

    EXPECT_EQ(Geofence::Result::Success, geofence->upload_geofence(polygons));

    EXPECT_EQ(Geofence::Result::Success, geofence->clear_geofence());
}

Geofence::Point add_point(double latitude_deg, double longitude_deg)
{
    Geofence::Point new_point;
    new_point.latitude_deg = latitude_deg;
    new_point.longitude_deg = longitude_deg;
    return new_point;
}
