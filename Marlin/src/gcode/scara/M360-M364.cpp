/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(SCARA_CALIBRATION)

#include "../gcode.h"
#include "../../module/scara.h"
#include "../../module/motion.h"
#include "../../module/planner.h"

inline bool SCARA_move_to_cal(const uint8_t theta, const uint8_t psi) {
  if (marlin.isRunning()) {
    forward_kinematics(theta, psi);
    motion.blocking_move_xy(motion.cartes);
    return true;
  }
  return false;
}

/**
 * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 */
bool GcodeSuite::M360() {
  SERIAL_ECHOLNPGM(" Cal: Theta 0");
  return SCARA_move_to_cal(0, 120);
}

/**
 * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 */
bool GcodeSuite::M361() {
  SERIAL_ECHOLNPGM(" Cal: Theta 90");
  return SCARA_move_to_cal(90, 40);
}

/**
 * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 */
bool GcodeSuite::M362() {
  SERIAL_ECHOLNPGM(" Cal: Psi 0");
  return SCARA_move_to_cal(60, 180-60);
}

/**
 * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 */
bool GcodeSuite::M363() {
  SERIAL_ECHOLNPGM(" Cal: Psi 90");
  return SCARA_move_to_cal(50, 90-50);
}

/**
 * M364: SCARA calibration: Move to cal-position PsiC (90 deg to Theta calibration position)
 */
bool GcodeSuite::M364() {
  SERIAL_ECHOLNPGM(" Cal: Theta-Psi 90");
  return SCARA_move_to_cal(45, 90);
}

/**
 * G6: SCARA Direct Joint Move (bypass inverse kinematics)
 *
 * Usage:
 *   G6 [T<theta_deg>] [P<psi_deg>] [A<theta_deg>] [B<psi_deg>] [Z<z_mm>] [E<r_deg>] [F<feedrate>]
 */
void GcodeSuite::G6() {
  if (motion.gcode_motion_ignored()) return;

  #if HAS_DIST_MM_ARG
    const xyze_float_t cart_dist_mm{0};
  #endif

  abce_pos_t target = planner.get_axis_positions_mm();

  // Joint 1 (Theta / A-Axis)
  if (parser.seen('T')) target.a = parser.value_float();
  else if (parser.seen('A')) target.a = parser.value_float();

  // Joint 2 (Psi / B-Axis)
  if (parser.seen('P')) target.b = parser.value_float();
  else if (parser.seen('B')) target.b = parser.value_float();

  // Z-Axis
  if (parser.seen('Z')) target.c = parser.value_linear_units();

  // E-Axis (R-Axis / End-effector)
  if (parser.seen('E') || parser.seen('R')) target.e = parser.value_float();

  // Feedrate
  feedRate_t fr_mm_s = motion.feedrate_mm_s;
  if (parser.seen('F')) {
    fr_mm_s = MMM_TO_MMS(parser.value_feedrate());
    motion.feedrate_mm_s = fr_mm_s;
  }

  // Directly queue joint angles into planner without inverse kinematics
  planner.buffer_segment(target OPTARG(HAS_DIST_MM_ARG, cart_dist_mm), fr_mm_s, motion.extruder);
  planner.synchronize();

  // Update logical Cartesian coordinates using forward kinematics
  forward_kinematics(target.a, target.b);
  motion.position.x = motion.cartes.x;
  motion.position.y = motion.cartes.y;
  motion.position.z = target.c;
  motion.position.e = target.e;
}

#endif // SCARA_CALIBRATION

