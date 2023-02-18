// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.misc.InverseKinematics;
import frc.robot.subsystems.ArmShoulder;

public class ArmShoulderCommand extends CommandBase {
  /** Creates a new ArmPivotCommand. */
  private ArmShoulder subsystem;
  private InverseKinematics IK;
  public ArmShoulderCommand(ArmShoulder subsystem, InverseKinematics IK) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.IK = IK;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.setPosition(IK.getShoulderAngle()/((72/22)*30));
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
