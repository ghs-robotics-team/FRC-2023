// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoPickupCubeCommand extends CommandBase {
  private NetworkTableEntry err = NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("tx");
  private DriveTrain drive;
  private ProfiledPIDController pid = new ProfiledPIDController(0.3819718634, 0, 0, new Constraints(0.3, 0.1));
  private double minInput = 0.06;
  /** Creates a new AutoPickupCubeCommand. */
  public AutoPickupCubeCommand(DriveTrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("pipeline").setDouble(1);
    pid.setGoal(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double spd = pid.calculate(err.getDouble(0)*Math.PI/180);
    spd = Math.copySign(Math.max(Math.abs(spd),minInput), spd);
    drive.tankdrive(spd, -spd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankdrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atGoal();
  }
}
