// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.helper.SetPoints;

public class MoveArmCommand extends CommandBase {
  /** Creates a new MoveArmCommand. */
  private Joystick secondary;
  private SetPoints setPoint;
  private boolean toggle = false;
  private boolean pressed = false;
  public MoveArmCommand(Joystick secondary) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.secondary = secondary;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(secondary.getPOV() == 90){
      setPoint = SetPoints.Home;//dpad right
    }
    if(secondary.getPOV() == 270){
      setPoint = SetPoints.GrabIntake;//dpad left
    }
    if(secondary.getPOV() == 180){
      setPoint = SetPoints.GrabGround;//dpad down
    }
    if(secondary.getPOV() == 0){
      setPoint = SetPoints.GrabSubstation;//dpad up
    }
    if(secondary.getRawButton(4)){
      setPoint = SetPoints.PlaceHigh;//y button
    }
    if(secondary.getRawButton(2)){
      setPoint = SetPoints.PlaceMid;//b button
    }
    if(secondary.getRawButton(1)){
      setPoint = SetPoints.PlaceLow;//a button
    }
    if(secondary.getRawButton(3) && !toggle){
      pressed = !pressed;
      SetPoints.setCubeMode(pressed);
    }
    toggle = secondary.getRawButton(3);
    OperatorConstants.ShoulderTargetAngle = setPoint.getShoulderAngle();
    OperatorConstants.ElbowTargetAngle = setPoint.getElbowAngle();
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
