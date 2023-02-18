// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmBrake extends SubsystemBase {

  private DoubleSolenoid shoulderBrake = new DoubleSolenoid(6, PneumaticsModuleType.REVPH, 2, 3);
  private DoubleSolenoid elbowBrake = new DoubleSolenoid(6, PneumaticsModuleType.REVPH, 4, 5);


  /** Creates a new Claw. */
  public ArmBrake() {
    
  }

  public void brakeShoulder(){
    shoulderBrake.set(Value.kForward);
  }
  public void releaseShoulder(){
    shoulderBrake.set(Value.kReverse);
  } 
  public void brakeElbow(){
    elbowBrake.set(Value.kForward);
  }
  public void releaseElbow(){
    elbowBrake.set(Value.kReverse);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  }
