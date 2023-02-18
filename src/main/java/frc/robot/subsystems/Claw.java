// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  private DoubleSolenoid claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

  /** Creates a new Claw. */
  public Claw() {
    
  }

  public void openClaw(){
    claw.set(Value.kForward);
  }   
  public void closeClaw(){
    claw.set(Value.kReverse);
  }   
  public void killClaw(){
    claw.set(Value.kOff);
  }   
  
  public void toggleClaw(){
    claw.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  }
