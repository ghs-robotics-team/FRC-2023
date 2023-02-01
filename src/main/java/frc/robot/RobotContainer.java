// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.auto.AutoAlignArmToHeightCommand;
import frc.robot.commands.auto.AutoAlignRobotToTapeCommand;
import frc.robot.commands.auto.AutoExtendArmToDistCommand;
import frc.robot.commands.auto.ReleaseClawCommand;
import frc.robot.commands.teleop.ArmExtensionCommand;
import frc.robot.commands.teleop.ArmPivotCommand;
import frc.robot.commands.teleop.AutoBalanceCommand;
import frc.robot.commands.teleop.ClawCommand;
import frc.robot.commands.teleop.ElevatorCommand;
import frc.robot.commands.teleop.RotateTurret;
import frc.robot.commands.teleop.TankDrive;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.Balance;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Subsystems
  private ArmExtension armExtension = new ArmExtension();
  private ArmPivot armPivot = new ArmPivot();
  private Claw claw = new Claw();
  private DriveTrain driveTrain = new DriveTrain();
  private Elevator elevator = new Elevator();
  private Balance balance = new Balance();
  private Turret turret = new Turret();

  //Joysticks
  private Joystick joystick_left = new Joystick(0);
  private Joystick joystick_right = new Joystick(1);
  private Joystick secondarycontroller = new Joystick(2);
  private JoystickButton autoBalanceButton = new JoystickButton(secondarycontroller, 5);

  //Commands
  private ArmExtensionCommand armExtensionCommand = new ArmExtensionCommand(armExtension, secondarycontroller);
  private ArmPivotCommand armPivotCommand = new ArmPivotCommand(armPivot, secondarycontroller, joystick_right);
  private ClawCommand clawCommand = new ClawCommand(claw, secondarycontroller);
  private ElevatorCommand elevatorCommand = new ElevatorCommand(elevator, secondarycontroller);
  private RotateTurret rotateTurret = new RotateTurret(turret, secondarycontroller);
  private TankDrive tankDrive = new TankDrive(driveTrain, joystick_left, joystick_right);
  private AutoBalanceCommand autoBalance = new AutoBalanceCommand(driveTrain, balance);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  public void setup(){
    // armExtension.setDefaultCommand(armExtensionCommand);
    armPivot.setDefaultCommand(armPivotCommand);
    claw.setDefaultCommand(clawCommand);
    // elevator.setDefaultCommand(elevatorCommand);
    // turret.setDefaultCommand(rotateTurret);
    driveTrain.setDefaultCommand(tankDrive);
    autoBalanceButton.whileHeld(autoBalance);

    Command autoPlaceConeCommand = new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoAlignRobotToTapeCommand(driveTrain),
        new AutoAlignArmToHeightCommand(armPivot),
        new AutoExtendArmToDistCommand(armExtension)
      ),
      new ReleaseClawCommand(claw),
      new ParallelCommandGroup(
        //reset extend to 0
        //reset rotation to pointing up
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
