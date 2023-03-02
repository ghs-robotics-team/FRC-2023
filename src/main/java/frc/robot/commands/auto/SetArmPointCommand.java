// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.misc.InverseKinematics;
import frc.robot.helper.SetPoints;
import frc.robot.Constants.OperatorConstants;

public class SetArmPointCommand extends CommandBase {
  /** Creates a new SetArmPointCommand. */
  private InverseKinematics IK;
  private SetPoints target;
  public SetArmPointCommand(InverseKinematics IK, SetPoints target) {
    this.IK = IK;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IK.setXY(target.x, target.y);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (OperatorConstants.ElbowCorrect && OperatorConstants.ShoulderCorrect);
  }
}
