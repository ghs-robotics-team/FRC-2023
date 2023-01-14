// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private TalonFX frontLeft = new TalonFX(0);
  private TalonFX frontRight = new TalonFX(0);
  private TalonFX backLeft = new TalonFX(0);
  private TalonFX backRight = new TalonFX(0);
  private DifferentialDrive drivetrain;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    MotorControllerGroup left = new MotorControllerGroup(backLeft, frontLeft);
    MotorControllerGroup right = new MotorControllerGroup(backRight, frontRight);
    drivetrain = new DifferentialDrive(left, right);
  }
  public void tankdrive(double left, double right){
    drivetrain.tankDrive(left, right);
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
