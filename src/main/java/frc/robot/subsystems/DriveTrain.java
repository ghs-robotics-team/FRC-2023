// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private TalonFX frontLeft = new TalonFX(0);
  private TalonFX frontRight = new TalonFX(2);
  private TalonFX backLeft = new TalonFX(1);
  private TalonFX backRight = new TalonFX(3);
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
  }
  public void tankdrive(double left, double right){
    frontLeft.set(TalonFXControlMode.PercentOutput,left);
    frontRight.set(TalonFXControlMode.PercentOutput,right);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
