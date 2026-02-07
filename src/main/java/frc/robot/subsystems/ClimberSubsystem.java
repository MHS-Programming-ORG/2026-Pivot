// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;

public class ClimberSubsystem extends SubsystemBase {
  TalonFX TalonMotor1;
  TalonFX TalonMotor2;
  private final PIDController turnPID = new PIDController(0, 0, 0);

  public ClimberSubsystem() {
      TalonMotor1 = new TalonFX(9);
      TalonMotor2 = new TalonFX(15);
      turnPID.setTolerance(0);
  }

  public void goToHeight(double targetHeight) {
   // double output = turnPID.calculate(getMotor1Encoder(),targetHeight);
  }

  public double getMotor1Encoder() {
      return TalonMotor1.getRotorPosition().getValueAsDouble();
  }

  public void setClimberSpeed(double speed){
    TalonMotor1.set(speed);
    TalonMotor2.set(speed);
  }
  public boolean pidAtSetpoint() {
    return turnPID.atSetpoint();
  }
  public void stopMotor(){
    TalonMotor1.stopMotor();
    TalonMotor2.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getBoolean(getName(), false);
    SmartDashboard.getNumber("Motor1 Position", getMotor1Encoder());
  }
}

