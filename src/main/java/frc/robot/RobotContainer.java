// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmExtensionCommand;
import frc.robot.commands.ArmPivotCommand;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.RotateTurret;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private Turret turret = new Turret();

  //Joysticks
  private Joystick joystick_left = new Joystick(0);
  private Joystick joystick_right = new Joystick(1);
  private Joystick secondarycontroller = new Joystick(2);

  //Commands
  private ArmExtensionCommand armExtensionCommand = new ArmExtensionCommand(armExtension, secondarycontroller);
  private ArmPivotCommand armPivotCommand = new ArmPivotCommand(armPivot, secondarycontroller);
  private ClawCommand clawCommand = new ClawCommand(claw, secondarycontroller);
  private ElevatorCommand elevatorCommand = new ElevatorCommand(elevator, secondarycontroller);
  private RotateTurret rotateTurret = new RotateTurret(turret, secondarycontroller);
  private TankDrive tankDrive = new TankDrive(driveTrain, joystick_left, joystick_right);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

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
    armExtension.setDefaultCommand(armExtensionCommand);
    armPivot.setDefaultCommand(armPivotCommand);
    claw.setDefaultCommand(clawCommand);
    elevator.setDefaultCommand(elevatorCommand);
    turret.setDefaultCommand(rotateTurret);
    driveTrain.setDefaultCommand(tankDrive);
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
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
