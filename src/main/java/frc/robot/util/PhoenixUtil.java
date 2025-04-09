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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Rotation2d;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */

  private static Pose2d[] poseListScoreRight = {
    new Pose2d( // 19 right
      new Translation2d(3.832   , 5.073  ),
      new Rotation2d(-1.064)
    ),
    new Pose2d( //20r
      new Translation2d(5.104, 5.174),
      new Rotation2d(-2.166)
    ),
    new Pose2d(//21r
      new Translation2d(5.758, 4.109),
      new Rotation2d(-3.131)
    ),
    new Pose2d(//22r
      new Translation2d(5.251,2.915),
      new Rotation2d(2.120)
    ),
    new Pose2d(//17r
      new Translation2d(3.956,2.866),
      new Rotation2d(1.022)
    ),
    new Pose2d(//18r
      new Translation2d(3.210,3.935),
      new Rotation2d(0.0)
    ),
  };

  private static Pose2d[] poseListScoreLeft = {
    new Pose2d( // 19 left
      new Translation2d(4.033  , 5.241  ),
      new Rotation2d(-1.003)
    ),
    new Pose2d(//20l
      new Translation2d(5.315, 4.885),
      new Rotation2d(-2.249)
    ),
    new Pose2d(//21l
      new Translation2d(5.775, 3.758),
      new Rotation2d(3.104)
    ),    
    new Pose2d(//22l
      new Translation2d(4.858,2.765),
      new Rotation2d(2.004)
    ),
    new Pose2d(//17l
      new Translation2d(3.651,3.044),
      new Rotation2d(0.995)
    ),
    new Pose2d(//18l
      new Translation2d(3.210,4.260),
      new Rotation2d(0.0)
    )
  };

  private static Pose2d[] poseListSource = {
    new Pose2d(
      new Translation2d(0.6334888935089111, 6.715460300445557 ),
      new Rotation2d(2.191046014709366)
    ),
    new Pose2d(
      new Translation2d(0.7365333437919617, 1.2747132778167725 ),
      new Rotation2d(-2.224879057281016)
    ),
    new Pose2d(
      new Translation2d(15.904675483703613, 7.416162490844727),
      new Rotation2d(0.9403934936948359)
    ),
    new Pose2d(
      new Translation2d(16.028329849243164, 0.656446635723114),
      new Rotation2d(-0.9561312775434612)
    )
  };

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  public static Pose2d getClosestPose(Pose2d currPose, Supplier<Boolean> isLeft) {
    double min_distance = 99999;
    Pose2d min_distance_pose = new Pose2d();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
      if (isLeft.get().booleanValue()) {
        for (int i = 0; i < poseListScoreLeft.length; i++) {

          Pose2d poseRed = new Pose2d(
            new Translation2d(
              poseListScoreLeft[i].getX() + 8.57301,
              poseListScoreLeft[i].getY()
            ),
            poseListScoreLeft[i].getRotation()
          );

          double dist = currPose.getTranslation().getDistance(poseRed.getTranslation());
    
          if (dist < min_distance) {
            min_distance = dist;
            min_distance_pose = poseRed;
          }
        }
      } else {
        for (int i = 0; i < poseListScoreLeft.length; i++) {

          // 8.573 + (12.62 - 12.606)
          Pose2d poseRed = new Pose2d(
            new Translation2d(
              poseListScoreRight[i].getX() + 8.587,
              poseListScoreRight[i].getY()
            ),
            poseListScoreRight[i].getRotation()
          );

          double dist = currPose.getTranslation().getDistance(poseRed.getTranslation());
    
          if (dist < min_distance) {
            min_distance = dist;
            min_distance_pose = poseRed;
          }
        }
      }
    } else {
      if (isLeft.get().booleanValue()) {
        for (int i = 0; i < poseListScoreLeft.length; i++) {
          double dist = currPose.getTranslation().getDistance(poseListScoreLeft[i].getTranslation());

          if (dist < min_distance) {
            min_distance = dist;
            min_distance_pose = poseListScoreLeft[i];
          }
        }
      } else {
        for (int i = 0; i < poseListScoreRight.length; i++) {
          double dist = currPose.getTranslation().getDistance(poseListScoreRight[i].getTranslation());

          if (dist < min_distance) {
            min_distance = dist;
            min_distance_pose = poseListScoreRight[i];
          }
        }

      }
    }

    return min_distance_pose;
  }

  public static Pose2d returnSourceRotatedPose(Pose2d currPose) {
    double min_distance = 99999;
    Pose2d min_distance_pose = new Pose2d();

    for (int i = 0; i < poseListSource.length; i++) {
      double dist = currPose.getTranslation().getDistance(poseListSource[i].getTranslation());

      if (dist < min_distance) {
        min_distance = dist;
        min_distance_pose = poseListSource[i];
      }
    }

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
      return new Pose2d(
        currPose.getTranslation(),
        min_distance_pose.getRotation()
      );
   } else {
      return new Pose2d(
        currPose.getTranslation(),
        min_distance_pose.getRotation()
      );
    }

  }
  
  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static enum ReefTarget {
        L1(0.0, 3, 15.25 * 45/60), 
        L2(0.8693, 10, 9.157 * 45/60),
        L3(2.31918, 10, 9.3 * 45/60),
        L4(4.65, 5, 9.3 * 45/60);

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
