// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }
  
  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static enum ReefTarget {
        L1(0.0, 3, 15.25), 
        L2(0.6693, 10, 9.357),
        L3(2.31918, 10, 8.5),
        L4(4.6, 5, 8.8);

        public final double elevatorHeight;
        public final double outtakeSpeed;
        public final double wristPosition;

        private ReefTarget(double elevatorHeight, double outtakeSpeed, double wristPosition) {
            this.elevatorHeight = elevatorHeight;
            this.outtakeSpeed = outtakeSpeed;
            this.wristPosition = wristPosition;
        }
  }

}
