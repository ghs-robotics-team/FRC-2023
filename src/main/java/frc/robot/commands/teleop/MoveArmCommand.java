// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.misc.InverseKinematics;
import frc.robot.helper.SetPoints;

public class MoveArmCommand extends CommandBase {
  /** Creates a new MoveArmCommand. */
  private InverseKinematics IK;
  private Joystick secondary;
  private SetPoints setPoint;
  public MoveArmCommand(InverseKinematics IK, Joystick secondary) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.IK = IK;
    this.secondary = secondary;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(secondary.getRawButton(1)){
      setPoint = SetPoints.Home;//a button
    }
    if(secondary.getRawButton(2)){
      setPoint = SetPoints.Intake;//b button
    }
    if(secondary.getRawButton(3)){
      setPoint = SetPoints.PlaceHigh;//x button
    }
    if(secondary.getRawButton(4)){
      setPoint = SetPoints.PlaceMid;//y button
    }
    if(secondary.getRawButton(5)){
      setPoint = SetPoints.PlaceLow;//lb button
    }
    if(secondary.getRawButton(6)){
      setPoint = SetPoints.GrabSubstation;//rb button
    }
    IK.setXY(setPoint.x, setPoint.y);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
