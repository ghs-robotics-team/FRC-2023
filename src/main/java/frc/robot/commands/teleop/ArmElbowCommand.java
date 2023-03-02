// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.misc.InverseKinematics;
import frc.robot.subsystems.ArmElbow;

public class ArmElbowCommand extends CommandBase {
  /** Creates a new ArmElbowCommand. */
  private ShuffleboardTab tab = Shuffleboard.getTab("PID");
  private GenericEntry p = tab.add("Elbow P",1).getEntry();
  private GenericEntry i = tab.add("Elbow I",1).getEntry();
  private GenericEntry d = tab.add("Elbow D",1).getEntry();
  private ArmElbow elbowSubsystem;
  private double angleToTickFactor = 49.0*2048*17/22;
  private InverseKinematics IK;
  private ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, new Constraints(0.2,0.01));
  public ArmElbowCommand(ArmElbow elbow, InverseKinematics IK) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elbow);
    this.elbowSubsystem = elbow;
    this.IK = IK;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.setP(p.getDouble(0));
    pid.setI(i.getDouble(0));
    pid.setD(d.getDouble(0));
    double targetPos = IK.getElbowAngle()*angleToTickFactor;
    double speed = pid.calculate(elbowSubsystem.getPos(), targetPos);
    elbowSubsystem.setSpeed(Math.max(Math.min(speed,0.2),-0.2));
    SmartDashboard.putNumber("Elbow Target", targetPos);
    SmartDashboard.putNumber("Elbow Position", elbowSubsystem.getPos());
    SmartDashboard.putNumber("Elbow PID Speed Output", speed);
    if(Math.abs(targetPos-elbowSubsystem.getPos()) < 2*Math.PI/360*angleToTickFactor){
      OperatorConstants.ShoulderCorrect = true;
    }else{
      OperatorConstants.ShoulderCorrect = false;
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
