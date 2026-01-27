// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Drivetrain;


public class RobotContainer {

  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
  // public static CommandPS5Controller debugController = new CommandPS5Controller(2);

  public static XboxController testController = new XboxController(2);
  public static Drivetrain drivetrain =
      new Drivetrain(new File(Filesystem.getDeployDirectory(), "swerve/"));

  // Subsystems
  
  public static boolean isDriverMode = false;

  public RobotContainer() {

    // auto.setupPathPlanner();
    // auto.setupAutoTab();
    setupDriverTab();
    
    drivetrain.setDefaultCommand(
        Commands.run(
            () -> {
              drivetrain.setInputFromController(driverController);
            },
            drivetrain));

    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {

    // ************* DRIVER CONTROLLER BINDINGS **************** //
    driverController
        .L2()
        .onTrue(Commands555.disableFieldRelative())
        .onFalse(Commands555.enableFieldRelative());
    
    
    
    

    // driverController
    //     .triangle()
    //     .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(0), false));
    // driverController
    //     .circle()
    //     .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(-90), false));
    // driverController
    //     .cross()
    //     .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false));
    // driverController
    //     .square()
    //     .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(90), false));


    BooleanSupplier isOnBlueAlliance = () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue;

    driverController
      .triangle()
      .onTrue(new ConditionalCommand(
        Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(0), false), 
        Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false), 
        isOnBlueAlliance));
      
    driverController
      .circle()
      .onTrue(new ConditionalCommand(
        Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(-90), false), 
        Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(90), false), 
        isOnBlueAlliance));
    
    driverController
      .cross()
      .onTrue(new ConditionalCommand(
        Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false), 
        Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(0), false), 
        isOnBlueAlliance));

    driverController
        .square()
        .onTrue(new ConditionalCommand(
          Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(90), false), 
          Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(-90), false),
          isOnBlueAlliance));

    driverController
        .touchpad()
        .onTrue(Commands555.zeroGyro()
                .ignoringDisable(true));
    driverController.PS().onTrue(Commands555.lockDrive());

    
    
    if (isOnBlueAlliance.getAsBoolean()) {
      driverController.povDown().onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(60), false));
      driverController.povRight().onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(300), false));
      driverController.povUp().onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false));

    } else {
      driverController.povDown().onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(-120), false));
      driverController.povRight().onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(120), false));
      driverController.povUp().onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(0), false));
    }    
                    
  }
  private void configureOperatorBindings() {

  }

  public void setupDriverTab() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");


    driverTab.addDouble("Time Remaining", () -> { return (int) Timer.getMatchTime();});


 
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return Commands555.setAutoPose("2");
    // return auto.getAutoCommand();
    return Commands.runOnce(() -> {});
  }
}