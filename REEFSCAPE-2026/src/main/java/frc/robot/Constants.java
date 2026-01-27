// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.path.PathConstraints;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Drivetrain;
import java.awt.geom.Point2D;


public final class Constants {

  public static class FieldConstants {
    public static final double SPEAKER_SCORE_X_OFFSET = 6.0;
    
    
    public static final Pose2d RED_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(652.3 - SPEAKER_SCORE_X_OFFSET), Units.inchesToMeters(218.42), new Rotation2d());
    public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(-1.5 + SPEAKER_SCORE_X_OFFSET), Units.inchesToMeters(218.42), new Rotation2d());
    public static final double SPEAKER_SCORE_HEIGHT = Units.inchesToMeters(80.511811);
    
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {

    public static double kp = 0.4;
    public static double kd = 0;
    public static double ki = 0.01;


    public static final double DRIVE_STATOR_CURRENT_LIMIT = 80;
    public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 80;
    public static final CurrentLimitsConfigs DRIVE_CONFIG = new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(80);
      
    public static final double DRIVE_KS = 0.12;
    public static final double DRIVE_KV = 2.8;
    public static final double DRIVE_KA = 0.17;
    public static final double BUMPER_WIDTH = Units.inchesToMeters(3); // TODO get real width

    public static final boolean IS_OPEN_LOOP = false;

    public static final double MAX_VOLTAGE_V = 12.0;
    public static final double MAX_SPEED = Units.feetToMeters(13.7);

    public static final double MAX_ROT_SPEED = 2 * Math.PI;
    // Max Acceleration in M/s^2
    // public static final double MAX_ACCELERATION = Units.feetToMeters(6);
    // Max angular acceleration in Rad/S^2
    // public static final double MAX_ANGULAR_ACCELERATION = 1.5;

    public static final double DRIVE_BASE_RADIUS = 0.43;

    public static final Pose2d EDGE_OF_DRIVEBASE =
        new Pose2d(0, DRIVE_BASE_RADIUS + BUMPER_WIDTH, new Rotation2d());
    
    public static final double ANGLE_DEADBAND = 2.5;
  }
}