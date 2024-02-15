// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.controls.DualShock4Controller;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveCommand;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser = RobotContainer.initAutoChooser();

  DualShock4Controller pilot = new DualShock4Controller(0);
  public static DriveSubsystem drive = new DriveSubsystem();
  public static VisionSubsystem vision = new VisionSubsystem();

  private final SendableChooser<Double> speedChooser = new SendableChooser<>();
  
  public RobotContainer() {  
    drive.setDefaultCommand(new DriveCommand(() -> pilot.getLeftAxes(), () -> pilot.getRightAxes(), () -> speedChooser.getSelected()));
    bindPilotControls();
    initSpeedChooser();
  }

  private void bindPilotControls() {
   
    pilot.cross
      .onTrue(new InstantCommand(() -> vision.enableLed()))
      .whileTrue(new DriveCommand(
        () -> new Translation2d(0.0, 0.0),
        () -> new Translation2d(vision.getXError(), 0.0),
        () -> speedChooser.getSelected()
      ))
      .onFalse(new InstantCommand(() -> vision.disableLed()));


    pilot.ps
      .onTrue(new InstantCommand(() -> drive.zeroGyro())); 
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static SendableChooser<Command> initAutoChooser() {
    
    NamedCommands.registerCommand("LED On", new InstantCommand(() -> vision.enableLed()));
    NamedCommands.registerCommand("LED Off", new InstantCommand(() -> vision.disableLed()));
  

    var autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    return autoChooser;
  }

  public void initSpeedChooser() {

    speedChooser.setDefaultOption("25%", 0.25);

    speedChooser.addOption("50%", .5);
    speedChooser.addOption("75%", .75);
    speedChooser.addOption("100%", 1.0);

    SmartDashboard.putData("Robot Speed", speedChooser);
  }
}
