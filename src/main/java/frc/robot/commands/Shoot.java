// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter shooter;

  private final Supplier<Double> triggerValue;
  private boolean isDone = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(Shooter shooter, Supplier<Double> triggerFunction) {

    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    this.triggerValue = triggerFunction;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterSpeed = triggerValue.get() * 6000;

    shooter.setVelocity(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
