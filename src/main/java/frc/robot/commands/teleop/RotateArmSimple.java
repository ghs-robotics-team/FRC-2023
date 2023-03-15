// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmBrake;


public class RotateArmSimple extends CommandBase {
  /** Creates a new MoveArmCommand. */
  private Joystick secondary;
  private ArmBrake brakeSubsystem;

  public RotateArmSimple(ArmBrake brakeSubsystem, Joystick secondary) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.brakeSubsystem = brakeSubsystem;
    this.secondary = secondary;

    addRequirements(this.brakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    brakeSubsystem.releaseElbow();
    brakeSubsystem.releaseShoulder();
    double elbowInput = secondary.getRawAxis(0);
    if(Math.abs(elbowInput) < 0.2){
      elbowInput = 0;
    }
    double shoulderInput = secondary.getRawAxis(4);
    if(Math.abs(shoulderInput) < 0.2){
      shoulderInput = 0;
    }
    OperatorConstants.ElbowTargetAngle+=elbowInput*0.03490658503988659153847381536977/3;
    OperatorConstants.ShoulderTargetAngle+=shoulderInput*0.03490658503988659153847381536977/3;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
