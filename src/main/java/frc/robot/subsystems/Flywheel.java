// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferences.PrefDouble;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Flywheel extends SubsystemBase implements Reportable {

  private TalonFX flywheel;
  public PrefDouble flywheelSetpoint = new PrefDouble("Flywheel Setpoint", 0);
  public PrefDouble kFlywheelP = new PrefDouble("Flywheel kP", 0);
  public PrefDouble kFlywheelF = new PrefDouble("Flywheel kF", 0);

  private double voltage;
  private double velocity;

  public Flywheel() {
    flywheel = new TalonFX(20);
    init();
  }

  public void init() {
    flywheel.setNeutralMode(NeutralMode.Brake);
    flywheel.setSelectedSensorPosition(0);

    kFlywheelP.loadPreferences();
    kFlywheelF.loadPreferences();

    flywheel.config_kP(0, kFlywheelP.get());
    flywheel.config_kF(0, kFlywheelF.get());

    flywheel.configNeutralDeadband(0.05);
    flywheel.configVoltageCompSaturation(11, 0);
    flywheel.enableVoltageCompensation(true);
  }

  public void refreshPID() {
    kFlywheelP.loadPreferences();
    kFlywheelF.loadPreferences();

    flywheel.config_kP(0, kFlywheelP.get());
    flywheel.config_kF(0, kFlywheelF.get());
  }

  public void setVelocity(double velocity) {
    flywheel.set(ControlMode.Velocity, velocity);
  }

  public void setVelocityZero() {
    setVelocity(0);
  }

  public void disableMotor() {
    flywheel.set(ControlMode.PercentOutput, 0);
    voltage = 0;
  }  

  public Command increment(double seconds){
    return new ParallelCommandGroup(
      Commands.repeatingSequence(
        Commands.waitSeconds(seconds),
        Commands.runOnce(() -> voltage += 0.05)
      ),
      Commands.run(() -> flywheel.set(ControlMode.PercentOutput, voltage))
    ).finallyDo((xD) -> disableMotor());
  }

  public Command incrementVelocity(double targetVelocity, double delay){
    return 
      new ParallelDeadlineGroup(
        Commands.waitUntil(() -> velocity >= targetVelocity),
        Commands.runOnce(() -> velocity = 0),
        Commands.repeatingSequence(
          Commands.waitSeconds(delay),
          Commands.runOnce(() -> velocity += (targetVelocity / 5))
        ),
        Commands.run(() -> flywheel.set(ControlMode.Velocity, velocity))
      );
  }

  public Command decrement(double seconds){
    return new ParallelCommandGroup(
      Commands.repeatingSequence(
        Commands.waitSeconds(seconds),
        Commands.runOnce(() -> voltage -= 0.05)
      ),
      Commands.run(() -> flywheel.set(ControlMode.PercentOutput, voltage))
    ).finallyDo((xD) -> disableMotor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void reportToSmartDashboard(LOG_LEVEL priority) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void initShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Flywheel");

    tab.addNumber("Flywheel Velocity", flywheel::getSelectedSensorVelocity);
    tab.addNumber("Flywheel Voltage", flywheel::getMotorOutputVoltage);
    tab.addNumber("Flywheel Output Percent", flywheel::getMotorOutputPercent);
    tab.addNumber("Flywheel Target Velocity", () -> velocity);
    
  }
}
