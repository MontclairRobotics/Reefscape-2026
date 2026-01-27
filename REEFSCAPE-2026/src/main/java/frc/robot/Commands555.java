package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;

public class Commands555 {
  private static double initTurnAngle;

  /**
   * Drive to a robot-relative point given a Translation2d & target Rotation2d.
   *
   * @param targetTranslation Field-relative Translation2d to drive the robot to.
   * @param theta             Target angle for end position.
   */
  // public static Command driveToRobotRelativePoint(
  //     Translation2d targetTranslation, Rotation2d theta) {
  //   Pose2d currentRobotPosition = RobotContainer.drivetrain.getSwerveDrive().getPose();
  //   Rotation2d currentOdometryHeading = RobotContainer.drivetrain.getSwerveDrive().getOdometryHeading();

  //   Translation2d targetTranslation2d = currentRobotPosition
  //       .getTranslation()
  //       .plus(targetTranslation.rotateBy(currentOdometryHeading));
  //   Pose2d botPose = new Pose2d(targetTranslation2d.getX(), targetTranslation2d.getY(), theta);

  //   // return AutoBuilder.pathfindToPose(
  //   //     botPose,
  //   //     AutoConstants.PATH_CONSTRAINTS,
  //   //     AutoConstants.GOAL_END_VELOCITY,
  //   //     AutoConstants.ROTATION_DELAY_DISTANCE);
  // }

  // Zero gyro
  public static Command zeroGyro() {
    return Commands.runOnce(() -> {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        //RobotContainer.drivetrain.getSwerveDrive().zeroGyro();
        RobotContainer.drivetrain.getSwerveDrive().resetOdometry(new Pose2d(RobotContainer.drivetrain.getSwerveDrive().getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      } else {
        RobotContainer.drivetrain.getSwerveDrive().zeroGyro();
      }
    });
  }



  public static Command lockDrive() {
    return Commands.runOnce(() -> {
      RobotContainer.drivetrain.getSwerveDrive().lockPose();
    }, RobotContainer.drivetrain);
  }

  /**
   * Drive to a field-relative point given a botPose
   *
   * @param botPose field-relative pose2d to drive the robot to.
   */
  // public static Command driveToFieldRelativePoint(Pose2d botPose) {
  //   return AutoBuilder.pathfindToPose(
  //       botPose,
  //       AutoConstants.PATH_CONSTRAINTS,
  //       AutoConstants.GOAL_END_VELOCITY,
  //       AutoConstants.ROTATION_DELAY_DISTANCE);
  // }


  public static Command setChassiSpeeds(ChassisSpeeds speeds) {
    return Commands.run(() -> {
      RobotContainer.drivetrain.setChassisSpeeds(speeds);
    });
  }

  public static Command driveOneMeter() {
    return Commands.run(RobotContainer.drivetrain::driveOneMeter);
  }


  // /**
  // * Robot relative or field relative depending on isFieldRelative. Input angle
  // MUST be between 0
  // and 360 degrees
  // * @param angle
  // * @return a command
  // */

  /***
   *
   * @param rot       a field relative Rotation2d supplier for the target angle
   * @param lockDrive should translational motion be locked during the command
   * @return a command that will lock angular control in favor of an angle that is
   *         provided
   *
   */
  public static Command alignToAngleFieldRelative(Supplier<Rotation2d> rot, boolean lockDrive) {
    Drivetrain drive = RobotContainer.drivetrain;
    return Commands.run(
        () -> {
          double thetaSpeed = drive
              .getSwerveDrive()
              .getSwerveController()
              .headingCalculate(
                  drive.getWrappedRotation().getRadians(), rot.get().getRadians());

          double xSpeed = 0;
          double ySpeed = 0;

          if (!lockDrive) {
            ySpeed = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.05)
                * DriveConstants.MAX_SPEED;
            xSpeed = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.05)
                * DriveConstants.MAX_SPEED;
          }

          if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)  {
            
             xSpeed *= -1;
             ySpeed *= -1;
          }

          RobotContainer.drivetrain.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed);
          // RobotContainer.drivetrain.drive(targetTranslation, thetaSpeed);
        },
        RobotContainer.drivetrain)
        .until(
            () -> {
              // boolean isAligned = Drivetrain.angleDeadband(
              //     RobotContainer.drivetrain.getWrappedRotation(),
              //         rot.get(), Rotation2d.fromDegrees(DriveConstants.ANGLE_DEADBAND));
              // System.out.println(rot.get().getDegrees());
              // System.out.println(isAligned);
              // System.out.println(RobotContainer.drivetrain.getWrappedRotation().getDegrees());

              return RobotContainer.drivetrain.getSwerveDrive().getSwerveController().thetaController.atSetpoint();
            });
  }

  public static Command alignContinuousFieldRelative(Supplier<Rotation2d> rot, boolean lockDrive) {
    Drivetrain drive = RobotContainer.drivetrain;
    return Commands.run(
        () -> {
          double thetaSpeed = drive
              .getSwerveDrive()
              .getSwerveController()
              .headingCalculate(
                  drive.getWrappedRotation().getRadians(), rot.get().getRadians());

          double xSpeed = 0;
          double ySpeed = 0;

          if (!lockDrive) {
            ySpeed = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.05)
                * DriveConstants.MAX_SPEED;
            xSpeed = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.05)
                * DriveConstants.MAX_SPEED;
          }

          if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)  {
           
             xSpeed *= -1;
             ySpeed *= -1;
          }

          RobotContainer.drivetrain.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed);
          // RobotContainer.drivetrain.drive(targetTranslation, thetaSpeed);
        },
        RobotContainer.drivetrain);
  }

    public static Command alignContinuousFieldRelativeWithDrive(Supplier<Rotation2d> rot, Translation2d speeds) {
    Drivetrain drive = RobotContainer.drivetrain;
    return Commands.run(
        () -> {
          double thetaSpeed = drive
              .getSwerveDrive()
              .getSwerveController()
              .headingCalculate(
                  drive.getWrappedRotation().getRadians(), rot.get().getRadians());

          RobotContainer.drivetrain.getSwerveDrive().drive(new Translation2d(speeds.getX(), speeds.getY()), thetaSpeed, false, true, new Translation2d() );
          // RobotContainer.drivetrain.drive(targetTranslation, thetaSpeed);
        },
        RobotContainer.drivetrain);
  }

  /***
   *
   * @param rot       a robot relative Rotation2d supplier for the target angle
   * @param lockDrive should translational motion be locked during the command
   * @return a command that will lock angular control in favor of an angle that is
   *         provided
   *
   */
  public static Command alignToAngleRobotRelative(Supplier<Rotation2d> rot, boolean lockDrive) {
    return alignToAngleFieldRelative(
        () -> {
          return Rotation2d.fromDegrees((initTurnAngle + rot.get().getDegrees()) % 360);
        },
        lockDrive)
        .beforeStarting(
            () -> {
              initTurnAngle = RobotContainer.drivetrain.getWrappedRotation().getDegrees();
            });
  }

  public static Command alignToAngleRobotRelativeContinuous(Supplier<Rotation2d> rot, boolean lockDrive) {
    return alignContinuousFieldRelative(
        () -> {
          return Rotation2d.fromDegrees(
              (RobotContainer.drivetrain.getWrappedRotation().getDegrees() + rot.get().getDegrees()) % 360);
        },
        lockDrive);
  }

  public static Command alignToAngleRobotRelativeContinuousWithDrive(Supplier<Rotation2d> rot, Translation2d speeds) {
    return alignContinuousFieldRelativeWithDrive(
        () -> {
          return Rotation2d.fromDegrees(
              (RobotContainer.drivetrain.getWrappedRotation().getDegrees() + rot.get().getDegrees()) % 360);
        },
        speeds);
  }


  /**
   * @param angle     the target angle in field space
   * @param lockDrive should translational motion be locked
   * @return
   */
  public static Command goToAngleFieldRelative(Rotation2d angle, boolean lockDrive) {
    return alignToAngleFieldRelative(
        () -> {
          return Drivetrain.wrapRotation(angle);
        },
        lockDrive);
  }

  /**
   * @param angle     the target angle in robot space
   * @param lockDrive should translational motion be locked
   * @return
   */
  public static Command goToAngleRobotRelative(Rotation2d angle, boolean lockDrive) {
    return alignToAngleRobotRelative(
        () -> {
          return angle;
        },
        lockDrive);
  }

  // public static Command alignToAmpAndShoot() {
  
  //   Pose2d drivePose = RobotContainer.drivetrain.getSwerveDrive().getPose();

  //   Translation2d targetTranslation = drivePose.getTranslation();
  //   targetTranslation = targetTranslation.plus(RobotContainer.shooterLimelight.getTargetPoseRobotSpace().getTranslation()).minus(new Translation2d(DriveConstants.BUMPER_WIDTH + DriveConstants.DRIVE_BASE_RADIUS, 0));

  //   Rotation2d rot = Drivetrain.flipAngle(Rotation2d.fromDegrees(270));

  //   // targetTranslation.plus(AutoConstants.TRANSLATION_FROM_AMP);

  //   List<Translation2d> points = PathPlannerPath.bezierFromPoses(
  //     drivePose,
  //     new Pose2d(targetTranslation, rot)
  //   );

  //   PathPlannerPath path = new PathPlannerPath(points, AutoConstants.PATH_CONSTRAINTS, new GoalEndState(0, rot));

  //   return Commands.sequence(
  //     Commands.parallel(
  //       AutoBuilder.followPath(path),
  //       setSprocketAngle(ArmConstants.AMP_SCORE_ANGLE),
  //       spinUpShooter(ShooterConstants.AMP_EJECT_SPEED_TOP, ShooterConstants.AMP_EJECT_SPEED_BOTTOM)
  //     ),
  //     shoot(ShooterConstants.AMP_EJECT_SPEED_TOP, ShooterConstants.AMP_EJECT_SPEED_BOTTOM, ShooterConstants.TRANSPORT_SPEED)
  //   );
  // }


  public static Command sysIdDrive() {
    SysIdRoutine routine = RobotContainer.drivetrain.getSysIdDrive();
    return Commands.sequence(
      routine.dynamic(Direction.kForward),
      waitForTime(5),
      routine.dynamic(Direction.kReverse),
      waitForTime(5),
      routine.quasistatic(Direction.kForward),
      waitForTime(5),
      routine.quasistatic(Direction.kReverse),
      waitForTime(5)
    );
  }

  // public static Command addVisionMeasurement() {
  //   return Commands.runOnce(() -> {
  //     LimelightHelpers.PoseEstimate targetPose = RobotContainer.shooterLimelight.getAdjustedPose();
      
  //     RobotContainer.drivetrain.addVisionMeasurement(
  //       targetPose.pose,
  //       targetPose.timestampSeconds,
  //       RobotContainer.shooterLimelight.getVisionStdDevs(targetPose)
  //     );
  //   });
  // }



  // public static Command testPipeSwitch(Limelight camera, DetectionType pipe) {
  // Timer timer = new Timer();
  // return Commands.run(() -> {
  // timer.reset();
  // timer.start();
  // camera.setPipelineTo(pipe);
  // // System.out.println(Timer.getFPGATimestamp());
  // }).until(() -> {return camera.getPipelineType() == pipe;}).finallyDo(() -> {
  // System.out.println(timer.get());
  // timer.stop();
  // });
  // }
  /**



  /***
   * @param limey
   * @return A command that drives to the currently targeted april tag
   */
  // public static Command driveToAprilTag(Limelight limey) {
  // double[] aprilTagPoseArray =
  // LimelightHelpers.getLimelightNTDoubleArray(limey.getName(),
  // "targetpose_robotspace");
  // Pose2d aprilTagPose = new Pose2d(new Translation2d(aprilTagPoseArray[0],
  // aprilTagPoseArray[1]), new Rotation2d(aprilTagPoseArray[5]));
  // Pose2d botPose =
  // aprilTagPose.relativeTo(DriveConstants.EDGE_OF_DRIVEBASE);
  // this work the way I think it does
  // return driveToFieldRelativePoint(botPose);

  // }

  /***
   * Enables field relative mode
   * 
   * @return a command that enables field relative control
   */
  public static Command enableFieldRelative() {
    return Commands.runOnce(RobotContainer.drivetrain::enableFieldRelative);
  }

  /**
   * @return a command to disable field relative control
   */
  public static Command disableFieldRelative() {
    return Commands.runOnce(RobotContainer.drivetrain::disableFieldRelative);
  }

  public static Command waitUntil(BooleanSupplier condition) {
    return new Command() {
      @Override
      public boolean isFinished() {
        return condition.getAsBoolean();
      }
    };
  }

  public static Command driveFromSpeeds(ChassisSpeeds speeds) {
    return Commands.runOnce(() -> RobotContainer.drivetrain.getSwerveDrive().drive(speeds));
  }

  public static Command driveFromSpeedsSet(ChassisSpeeds speeds) {
    // return Commands.run(() -> RobotContainer.drivetrain.getSwerveDrive().drive(speeds));
    return Commands.run(() -> {
      RobotContainer.drivetrain.drive(new Translation2d(2, 0), 0);
    });
  }

  public static Command waitForTime(double seconds) {
    return Commands.run(() -> {
    }).withTimeout(seconds);
  }

  // public static Command translateToAmpCommand() {

  // public static Command scoreSubwooferAtAngle() {
  //   return Commands.sequence(
  //       Commands.runOnce(() -> System.out.println("Shooting!")),
  //       setSprocketAngle(ArmConstants.SPEAKER_SCORE_ANGLE),
  //       waitUntil(() -> {
  //         return RobotContainer.sprocket.isAtAngle();
  //       }),
  //       shoot(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED,
  //           ShooterConstants.TRANSPORT_SPEED),
  //       Commands.runOnce(() -> System.out.println("Done Shooting!")),
  //       setSprocketAngle(ArmConstants.INTAKE_ANGLE));
  // }


  // public static Command signalAmp() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new FlashAnimation(Color.kOrange));
  // });
  // }

  // public static Command signalCoop() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new FlashAnimation(Color.kBlue));
  // });
  // }

  // // LED bits
  // public static Command celebrate() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new CelebrationAnimation());
  // });
  // }

  // public static Command ampItUp() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new FlashAnimation(Color.kYellow));
  // });
  // }

  // public static Command cooperatition() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new FlashAnimation(Color.kBlueViolet));
  // });
  // }
}