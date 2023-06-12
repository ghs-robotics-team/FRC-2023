// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmElbow;

public class ArmElbowCommand extends CommandBase {
  /** Creates a new ArmElbowCommand. */
  private ShuffleboardTab tab = Shuffleboard.getTab("PID");
  private GenericEntry p = tab.add("Elbow P",4).getEntry();
  private GenericEntry i = tab.add("Elbow I",0).getEntry();
  private GenericEntry d = tab.add("Elbow D",0).getEntry();
  private GenericEntry maxSpeedEntry = tab.add("Max Elbow Speed",0.7).getEntry();
  private GenericEntry maxAccelEntry = tab.add("Max Elbow Accel",0.8).getEntry();
  private ArmElbow elbowSubsystem;
  private double maxSpeed = 0.15;
  private double angleToTickFactor = OperatorConstants.AngleToTickElbow;
  private ProfiledPIDController pid = new ProfiledPIDController(12, 0, 0, new Constraints(0.25,0.25));
  public ArmElbowCommand(ArmElbow elbow) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elbow);
    this.elbowSubsystem = elbow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset(elbowSubsystem.getPos()/angleToTickFactor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(OperatorConstants.ElbowTargetAngle < 0.11){
      OperatorConstants.ElbowTargetAngle = 0.11;
    }
    if(OperatorConstants.StopMovingElbow){
      OperatorConstants.ElbowTargetAngle = elbowSubsystem.getPos()/angleToTickFactor;
      OperatorConstants.StopMovingElbow = false;
    }
    pid.setConstraints(new Constraints(maxSpeed, maxAccelEntry.getDouble(0.01)));
    pid.setP(p.getDouble(0));
    pid.setI(i.getDouble(0));
    pid.setD(d.getDouble(0));
    maxSpeed = maxSpeedEntry.getDouble(0.15);
    double targetPos = OperatorConstants.ElbowTargetAngle;
    double speed = pid.calculate(elbowSubsystem.getPos()/angleToTickFactor, targetPos);
    elbowSubsystem.setSpeed(Math.max(Math.min(speed,maxSpeed),-maxSpeed));
    if(Math.abs(targetPos-elbowSubsystem.getPos()/angleToTickFactor) < 1*Math.PI/180){
      OperatorConstants.ElbowCorrect = true;
    }else{
      OperatorConstants.ElbowCorrect = false;
    }
    SmartDashboard.putNumber("Elbow Target", targetPos);
    SmartDashboard.putNumber("Elbow Position", elbowSubsystem.getPos()/angleToTickFactor);
    SmartDashboard.putNumber("Elbow MOE",Math.abs(targetPos-elbowSubsystem.getPos()/angleToTickFactor)*180/Math.PI);
    SmartDashboard.putBoolean("Elbow Correct", OperatorConstants.ElbowCorrect);
    SmartDashboard.putNumber("Elbow PID Speed Output", speed);
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
