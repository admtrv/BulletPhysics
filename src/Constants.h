/*
 * Constants.h
 */

#pragma once

#include "math/Vec3.h"
#include "math/Constants.h"

namespace BulletPhysics {
namespace constants {

/*
 * Constants
 */

// physical
inline math::Vec3 GRAVITY{0.0, 0.0, -9.80665};

// conversion
inline constexpr double CELSIUS_TO_KELVIN = 273.15;

// ISA sea-level
inline constexpr double ISA_TROPOSPHERE_MAX = 11000.0;                      // m (troposphere height)
inline constexpr double ISA_BASE_TEMPERATURE = 15 + CELSIUS_TO_KELVIN;      // K (15 C)
inline constexpr double ISA_TEMPERATURE_LAPSE_RATE = 0.0065;                // K/m (temperature lapse rate)
inline constexpr double ISA_BASE_PRESSURE = 101325.0;                       // Pa
inline constexpr double ISA_BASE_DENSITY = 1.225;                           // kg/m^3 (dry air)
inline constexpr double ISA_SPEED_OF_SOUND = 340.294;                       // m/s

// gas constants (J/(kg*K))
inline constexpr double GAS_CONSTANT_DRY_AIR = 287.058;
inline constexpr double GAS_CONSTANT_WATER_VAPOR = 461.495;

// Tetens approximation constants
inline constexpr double TETENS_A = 17.27;
inline constexpr double TETENS_B = 35.85;
inline constexpr double TETENS_C = 0.61078;

// WGS 84 Ellipsoid constants
inline constexpr double WGS84_EARTH_ANGULAR_SPEED = 72.92115e-6;               // rad/s
inline constexpr double WGS84_EARTH_GRAVITATIONAL_CONSTANT = 3.986004418e14;   // GM (m^3/s^2)
inline constexpr double WGS84_EARTH_SEMI_MAJOR_AXIS = 6378137.0;               // m (a, or equatorial radius)
inline constexpr double WGS84_EARTH_SEMI_MINOR_AXIS = 6356752.314245;          // m (b, or polar radius)
inline constexpr double WGS84_EARTH_ECCENTRICITY_SQUARED = 6.69437999014e-3;   // e^2

/*
 * Default Parameters
 */

// atmosphere
inline constexpr double DEFAULT_RELATIVE_HUMIDITY = 0.0;        // % (0-100)
inline const math::Vec3 DEFAULT_WIND{0.0, 0.0, 0.0};            // m/s

// geographic
inline constexpr double DEFAULT_LATITUDE = 0.0;     // rad
inline constexpr double DEFAULT_LONGITUDE = 0.0;    // rad
inline constexpr double DEFAULT_ALTITUDE = 0.0;     // m

// projectile parameters
inline constexpr double DEFAULT_MASS = 1.0;
inline constexpr double DEFAULT_DIAMETER = 0.01;
inline constexpr double DEFAULT_AREA = math::constants::PI * DEFAULT_DIAMETER * DEFAULT_DIAMETER / 4;     // m^2
inline constexpr double DEFAULT_C_D = 0.5;
inline constexpr double DEFAULT_C_M_ALPHA = 4.0;
inline constexpr double DEFAULT_C_L_ALPHA = 0.10;
inline constexpr double DEFAULT_C_MAG_F = 0.10;

} // namespace constants
} // namespace BulletPhysics
