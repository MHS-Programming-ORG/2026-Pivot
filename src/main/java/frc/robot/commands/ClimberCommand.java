// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  ClimberSubsystem climberSubsystem;
  private final double height;

  public ClimberCommand(ClimberSubsystem newClimberSubsystem, double height) {
    this.height = height;
    climberSubsystem = newClimberSubsystem;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
   // climberSubsystem.goToHeight(height);
    climberSubsystem.setClimberSpeed(0.3);
    System.out.print("climberSubsystem");
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // return climberSubsystem.pidAtSetpoint();
    return climberSubsystem.getMotor1Encoder() > height;
    
  }
}
