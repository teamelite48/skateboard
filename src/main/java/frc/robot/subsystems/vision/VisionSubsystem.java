// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  final NetworkTableEntry tid = table.getEntry("tid");
  final NetworkTableEntry tx = table.getEntry("tx");
  final NetworkTableEntry ty = table.getEntry("ty");
  final NetworkTableEntry ledMode = table.getEntry("ledMode");
  final NetworkTableEntry pipeline = table.getEntry("pipeline");

  public VisionSubsystem() {
    initShuffleBoard();
  }

  @Override
  public void periodic() {
  }

  public long getTargetId() {
    return tid.getInteger(0);
  }

  public double getXError() {
    return (getXOffset() + VisionConfig.CAMERA_OFFSET) / VisionConfig.VISION_RANGE_DEGREES;
  }

  public void enableLed() {
    enableAprilTagPipeline();
    ledMode.setNumber(3);
  }

  public void disableLed() {
    ledMode.setNumber(1);
  }

  public void enableAprilTagPipeline() {
    pipeline.setNumber(0);
  }

  public void enableReflectiveTapePipeline(){
    pipeline.setNumber(1);
  }

  private double getXOffset() {
    return tx.getDouble(0.0);
  }

  private double getYOffset() {
    return ty.getDouble(0.0);
  }

  private void initShuffleBoard() {
    var tab = Shuffleboard.getTab("Vision");

    tab.addDouble("Target ID", () -> getTargetId())
      .withPosition(0, 0);

    tab.addDouble("X Offset", () -> getXOffset())
      .withPosition(1, 0);

    tab.addDouble("Y Offset", () -> getYOffset())
      .withPosition(2, 0);

    tab.addDouble("X Error", () -> getXError())
      .withPosition(3, 0);

    tab.addInteger("LED Mode", () -> ledMode.getInteger(0))
      .withPosition(4, 0);
  }
}
