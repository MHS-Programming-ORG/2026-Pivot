// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */

  TalonFX TalonMotor3;
  private final PIDController pivotPID = new PIDController(0, 0, 0);

  public PivotSubsystem() {
    TalonMotor3 = new TalonFX(15);
    pivotPID.setTolerance(0);
  }
  
  public void goToAngle(double targetAngle){
    double output = pivotPID.calculate(getMotor3Encoder(),targetAngle);
  }

  public double getMotor3Encoder() {
    return TalonMotor3.getRotorPosition().getValueAsDouble();
  }
  
  public void setPivotSpeed(double speed){
    TalonMotor3.set(speed);
  }

  public void resetAngle() {
    TalonMotor3.setPosition(0);
  }

   public boolean pidAtSetpoint() {
    return pivotPID.atSetpoint();
  }
   public void stopMotor(){
    TalonMotor3.stopMotor();
  }


  @Override
  public void periodic() {
    SmartDashboard.getBoolean(getName(), false);
    SmartDashboard.getNumber("Motor3 Position", getMotor3Encoder());
  }
}
