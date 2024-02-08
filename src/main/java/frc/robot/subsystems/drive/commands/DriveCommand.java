// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveCommand extends CommandBase {

  DriveSubsystem driveSubsystem = RobotContainer.drive;
  Supplier<Translation2d> translationSupplier;
  Supplier<Translation2d> rotationSupplier;

  public DriveCommand(Supplier<Translation2d> translationSupplier, Supplier<Translation2d> rotationSupplier) {

    addRequirements(driveSubsystem);

    this.translationSupplier = translationSupplier;
    this.rotationSupplier = rotationSupplier;
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    var translation = translationSupplier.get();
    var rotation = rotationSupplier.get();

    driveSubsystem.manualDrive(
      translation.getX(),
      translation.getY(),
      rotation.getX()
    );
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
