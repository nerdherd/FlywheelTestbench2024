// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Flywheel;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public Flywheel flywheel;

  private final CommandPS4Controller controller = new CommandPS4Controller(0);

  public RobotContainer() {
    flywheel = new Flywheel();
    configureBindings();
  }

  private void configureBindings() {
    controller.circle()
    .onTrue(Commands.runOnce(
      () -> { 
        flywheel.refreshPID();
        flywheel.flywheelSetpoint.loadPreferences();
        flywheel.setVelocity(flywheel.flywheelSetpoint.get());
      }
      ))
    .onFalse(Commands.runOnce(() -> flywheel.setVelocityZero()));

    controller.square().onTrue(Commands.runOnce(() -> flywheel.disableMotor()));
    controller.cross().whileTrue(
      Commands.runOnce(() -> flywheel.flywheelSetpoint.loadPreferences())
      .andThen(flywheel.incrementVelocity(flywheel.flywheelSetpoint.get(), 1))
    ).onFalse(Commands.runOnce(flywheel::disableMotor));

    controller.triangle()
      .whileTrue(flywheel.increment(5));
  }

  public void initShuffleboard() {
    flywheel.initShuffleboard();
  }

  // // The robot's subsystems and commands are defined here...
  // private final Flywheel m_exampleSubsystem = new Flywheel();

  // // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // /** The container for the robot. Contains subsystems, OI devices, and commands. */
  // public RobotContainer() {
  //   // Configure the trigger bindings
  //   configureBindings();
  // }

  // /**
  //  * Use this method to define your trigger->command mappings. Triggers can be created via the
  //  * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
  //  * predicate, or via the named factories in {@link
  //  * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
  //  * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
  //  * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
  //  * joysticks}.
  //  */
  // private void configureBindings() {
  //   // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  //   new Trigger(m_exampleSubsystem::exampleCondition)
  //       .onTrue(new ExampleCommand(m_exampleSubsystem));

  //   // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
  //   // cancelling on release.
  //   m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  // }

  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
