/*
 * Constants.h
 */

#pragma once

#include "math/Vec3.h"
#include "math/Constants.h"

namespace BulletPhysics {
namespace constants {

// physical
inline math::Vec3 GRAVITY{0.0, -9.80665, 0.0};

// conversion
inline constexpr double CELSIUS_TO_KELVIN = 273.15;     // between Celsius and Kelvin

// atmospheric constants (ISA model)
inline constexpr double TROPOSPHERE_MAX = 11000.0;                      // m (troposphere height)
inline constexpr double BASE_TEMPERATURE = 15 + CELSIUS_TO_KELVIN;       // K (15 C)
inline constexpr double BASE_ATMOSPHERIC_PRESSURE = 101325.0;           // Pa
inline constexpr double BASE_ATMOSPHERIC_DENSITY = 1.225;               // kg/m^3 (dry air)
inline constexpr double BASE_SPEED_OF_SOUND = 340.294;                  // m/s
inline constexpr double LAPSE_RATE = 0.0065;                            // K/m (temperature lapse rate)
inline constexpr double GAS_CONSTANT_DRY_AIR = 287.058;                 // J/(kg·K)

// humidity constants (Tetens)
inline constexpr double GAS_CONSTANT_WATER_VAPOR = 461.495;             // J/(kg·K)
inline constexpr double TETENS_A = 17.27;
inline constexpr double TETENS_B = 35.85;
inline constexpr double TETENS_C = 0.61078;

// Earth (WGS standard)
inline constexpr double EARTH_ANGULAR_SPEED = 72.92115e-6;              // rad/s
inline constexpr double EARTH_GRAVITATIONAL_CONSTANT = 3.986004418e14;  // GM (m^3/s^2)
inline constexpr double EARTH_SEMI_MAJOR_AXIS = 6378137.0;              // m (a, or equatorial radius)
inline constexpr double EARTH_SEMI_MINOR_AXIS = 6356752.314245;         // m (b, or polar radius)
inline constexpr double EARTH_ECCENTRICITY_SQUARED = 6.69437999014e-3;  // e^2

// default projectile constants
static constexpr double DEFAULT_DIAMETER = 0.00762;
static constexpr double DEFAULT_AREA = math::constants::PI * DEFAULT_DIAMETER * DEFAULT_DIAMETER / 4;     // m^2
static constexpr double DEFAULT_CD = 0.3;
static constexpr double DEFAULT_C_M_ALPHA = 4.0;
static constexpr double DEFAULT_C_L_ALPHA = 0.10;
static constexpr double DEFAULT_C_MAG_F = 0.10;

} // namespace constants
} // namespace BulletPhysics
