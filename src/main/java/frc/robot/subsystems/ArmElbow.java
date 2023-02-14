// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmElbow extends SubsystemBase {
  private TalonFX elbowMotor = new TalonFX(6);
  /** Creates a new ArmElbow. */
  public ArmElbow() {
    elbowMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setPosition(double rad){
    elbowMotor.set(TalonFXControlMode.MotionMagic,rad*325.949323);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
