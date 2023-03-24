// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoMoveDistCommand extends CommandBase {
  /** Creates a new AutoMoveDistCommand. */
  private DriveTrain drive;
  private double dist;
  //14 ft for mobility, 16 ft for intaking a cube, use Units.feetToMeters in the input
  private double leftSpeed;
  private double rightSpeed;
  private Pose2d initial;
  public AutoMoveDistCommand(DriveTrain drive, double dist, double leftSpeed, double rightSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
    this.dist = dist;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.initial = drive.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.tankdrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankdrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Dist Left", dist-Math.hypot(initial.getX()-drive.getPose().getX(), initial.getY()-drive.getPose().getY()));
    return Math.hypot(initial.getX()-drive.getPose().getX(), initial.getY()-drive.getPose().getY()) >= dist;
  }
}
