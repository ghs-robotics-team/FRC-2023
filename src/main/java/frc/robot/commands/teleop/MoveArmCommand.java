// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.helper.SetPoints;
import frc.robot.subsystems.Claw;

public class MoveArmCommand extends CommandBase {
  /** Creates a new MoveArmCommand. */
  private Joystick secondary;
  private Claw claw;
  private boolean toggle = false;
  private boolean pressed = false;
  private double runTimer = 0;
  private boolean toggleRun = false;
  public MoveArmCommand(Joystick secondary, Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.secondary = secondary;
    this.claw = claw;
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    OperatorConstants.ElbowTargetAngle = OperatorConstants.armSetPoint.getElbowAngle();
    OperatorConstants.ShoulderTargetAngle = OperatorConstants.armSetPoint.getShoulderAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(secondary.getPOV() == 90){
      OperatorConstants.armSetPoint = SetPoints.Home;//dpad right
      claw.run(0);
      OperatorConstants.ShoulderTargetAngle = OperatorConstants.armSetPoint.getShoulderAngle();
      OperatorConstants.ElbowTargetAngle = OperatorConstants.armSetPoint.getElbowAngle();
    }
    if(secondary.getPOV() == 270){
      OperatorConstants.armSetPoint = SetPoints.GrabIntake;//dpad left
      if(SetPoints.cubeMode){
        claw.run(-0.3);
      }
      OperatorConstants.ShoulderTargetAngle = OperatorConstants.armSetPoint.getShoulderAngle();
      OperatorConstants.ElbowTargetAngle = OperatorConstants.armSetPoint.getElbowAngle();
    }
    if(secondary.getPOV() == 180){
      OperatorConstants.armSetPoint = SetPoints.GrabSingle;//dpad down
      if(SetPoints.cubeMode){
        claw.run(-0.3);
      }
      OperatorConstants.ShoulderTargetAngle = OperatorConstants.armSetPoint.getShoulderAngle();
      OperatorConstants.ElbowTargetAngle = OperatorConstants.armSetPoint.getElbowAngle();
    }
    if(secondary.getPOV() == 0){
      OperatorConstants.armSetPoint = SetPoints.GrabSubstation;//dpad up
      if(SetPoints.cubeMode){
        claw.run(-0.3);
      }
      OperatorConstants.ShoulderTargetAngle = OperatorConstants.armSetPoint.getShoulderAngle();
      OperatorConstants.ElbowTargetAngle = OperatorConstants.armSetPoint.getElbowAngle();
    }
    if(secondary.getRawButton(4)){
      OperatorConstants.armSetPoint = SetPoints.PlaceHigh;//y button
      claw.run(0);
      OperatorConstants.ShoulderTargetAngle = OperatorConstants.armSetPoint.getShoulderAngle();
      OperatorConstants.ElbowTargetAngle = OperatorConstants.armSetPoint.getElbowAngle();
    }
    if(secondary.getRawButton(2)){
      OperatorConstants.armSetPoint = SetPoints.PlaceMid;//b button
      claw.run(0);
      OperatorConstants.ShoulderTargetAngle = OperatorConstants.armSetPoint.getShoulderAngle();
      OperatorConstants.ElbowTargetAngle = OperatorConstants.armSetPoint.getElbowAngle();
    }
    if(secondary.getRawButton(3) && !toggle){
      pressed = !pressed;
      SetPoints.setCubeMode(pressed);
      if(!pressed){
        claw.run(0);
      }
      OperatorConstants.ShoulderTargetAngle = OperatorConstants.armSetPoint.getShoulderAngle();
      OperatorConstants.ElbowTargetAngle = OperatorConstants.armSetPoint.getElbowAngle();
    }
    SmartDashboard.putBoolean("Cone Mode", !SetPoints.cubeMode);
    toggle = secondary.getRawButton(3);
    if(secondary.getRawAxis(2)>0.1){
      claw.openClaw();
    }
    if(secondary.getRawAxis(3)>0.1){
      claw.closeClaw();
    }
    if(secondary.getRawButton(5)){
      claw.openClaw();
      runTimer = 50;
      claw.run(0.1);
    }
    if(secondary.getRawButton(6) && !toggleRun){
      if(claw.runSpeed == 0){
        claw.run(-0.3);
      }else{
        claw.run(0);
      }
    }
    toggleRun = secondary.getRawButton(6);
    if(runTimer > 0){
      runTimer--;
    }else if(runTimer == 0){
      runTimer--;
      claw.run(0);
    }
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
