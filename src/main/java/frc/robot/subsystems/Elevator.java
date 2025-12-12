// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.OperatorConstants;
// import edu.wpi.first.wpilibj.Encoder;

// public class Elevator extends SubsystemBase {
//   /** Creates a new Elevator. */
//   TalonFX elMotor;

//   PIDController elPID ;
//   final PositionVoltage request;
//   public Encoder encoder;
//   private double target;

//   public Elevator() {
//     elMotor = new TalonFX(OperatorConstants.elKrakenId);
//     // encoder = new Encoder(0,1);

//     elMotor.setPosition(0);
//        var slot0Configs = new Slot0Configs();
//     slot0Configs.kV = 0;
//     slot0Configs.kA = 0;
//    slot0Configs.kP = 0.3;
//    slot0Configs.kI = 0;
//    slot0Configs.kD = 0.05;
//    slot0Configs.kG =0;
//    elMotor.getConfigurator().apply(slot0Configs);
//     request = new PositionVoltage(0).withSlot(0);


//   }

//   public void turnPoint(){
//     // System.out.println(elMotor.getRotorPosition().getValue());
//     if(Math.abs(target-elMotor.getRotorPosition().getValueAsDouble())>OperatorConstants.kElDeadBand){
//       elMotor.setControl(request.withPosition(target));
//     }
//     else{
//       elMotor.set(0);
//     }
    

//   }
//   public boolean setPoint(double goal){
//     target= goal;
//     System.out.println("hello");
    
//     return Math.abs(target-elMotor.getRotorPosition().getValueAsDouble())<OperatorConstants.kElDeadBand;
//   }

//   public void stopElevator(){
//     elMotor.set(0);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
