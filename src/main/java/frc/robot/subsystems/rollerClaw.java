// package frc.robot.subsystems;
// // // Copyright (c) FIRST and other WPILib contributors.
// // // Open Source Software; you can modify and/or share it under the terms of
// // // the WPILib BSD license file in the root directory of this project.



// import java.lang.management.MemoryType;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class rollerClaw extends SubsystemBase {
//   /** Creates a new rollerClaw. */
//    private SparkMax rolly1;
//    private SparkMax rolly2;
//    private RelativeEncoder cEncoder;
//    private SparkMaxConfig motorConfig;
//   //  DigitalInput limSwitch;
 

//   public rollerClaw() {
//     rolly1 = new SparkMax(28, MotorType.kBrushless);
//     rolly2= new SparkMax(29, MotorType.kBrushless);

//       // motorConfig = new SparkMaxConfig();
//       //   motorConfig.encoder
//       //   .positionConversionFactor(1)
//       //   .velocityConversionFactor(1);

//       //       motorConfig.closedLoop
//       //   .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//       //   // Set PID values for position control. We don't need to pass a closed loop
//       //   // slot, as it will default to slot 0.
//       //   .p(0.0000001)
//       //   .i(0)
//       //   .d(0);
//       //   // .outputRange(-1, 1);
//       //   // // Set PID values for velocity control in slot 1
//       //   // .p(0.0001, ClosedLoopSlot.kSlot1)
//       //   // .i(0, ClosedLoopSlot.kSlot1)
//       //   // .d(0, ClosedLoopSlot.kSlot1)
//       //   // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
//       //   // .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

//       //    rolly1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//       //    rolly2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//   }
//   public void spinIn(){
//     rolly1.set(-0.15);
//     rolly2.set(0.15);
//     // if(limSwitch.get()){
//     //   rolly1.set(10);
//     //   rolly2.set(-10);
//     // }
//     // else{
//     //   rolly1.set(0);
//     //   rolly2.set(0);
//     // }
//   }
//   public void spinOut(){
//     rolly1.set(0.32);
//     rolly2.set(-0.32);
//   }
//   public void stopSpin(){
//     rolly1.set(0);
//     rolly2.set(0);
//   }
//   public void spintogether(){
//     rolly1.set(0.31);
//     rolly2.set(0.31);
//   }
//   public void L1Rolly(){
//     rolly1.set(0.075);
//     rolly2.set(-0.075);
//   }


//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
