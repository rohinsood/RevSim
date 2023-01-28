// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.REVPhysicsSim;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxAbsoluteEncoder;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.CounterBase.EncodingType;
// import edu.wpi.first.wpilibj.simulation.EncoderSim;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;

// /**
//  * The VM is configured to automatically run this class, and to call the
//  * functions corresponding to
//  * each mode, as described in the TimedRobot documentation. If you change the
//  * name of this class or
//  * the package after creating this project, you must also update the
//  * build.gradle file in the
//  * project.
//  */
// public class Robot extends TimedRobot {
//   private final SendableChooser<Integer> m_chooser = new SendableChooser<>();

//   private final CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);

//   private SparkMaxPIDController pidController;
//   private SparkMaxAbsoluteEncoder absoluteEncoder;

//   private Joystick joystick;

//   public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  // private final Encoder encoder = new Encoder(0, 1);
  // private final EncoderSim encoderSim = new EncoderSim(encoder);

  // private final Mechanism2d mechanism2d = new Mechanism2d(180, 180);
  // private final MechanismRoot2d center = mechanism2d.getRoot("center", 45, 90);
  // private final MechanismLigament2d square = center.append(new MechanismLigament2d("square", 90, 0, 10, new Color8Bit(Color.kWhite)));

//   /**
//    * This function is run when the robot is first started up and should be used
//    * for any
//    * initialization code.
//    */
//   @Override
//   public void robotInit() {
//     m_chooser.setDefaultOption("case 1", 1);
//     joystick = new Joystick(0);

//     motor.restoreFactoryDefaults();

//     pidController = motor.getPIDController();
    
//     absoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    
//     pidController.setFeedbackDevice(absoluteEncoder);

//     kP = 6e-5;
//     kI = 0;
//     kD = 0;
//     kIz = 0;
//     kFF = 0.000015;
//     kMaxOutput = 1;
//     kMinOutput = -1;

//     // set PID coefficients
//     pidController.setP(kP);
//     pidController.setI(kI);
//     pidController.setD(kD);
//     pidController.setIZone(kIz);
//     pidController.setFF(kFF);
//     pidController.setOutputRange(kMinOutput, kMaxOutput);

//     // display PID coefficients on SmartDashboard
//     SmartDashboard.putNumber("P Gain", kP);
//     SmartDashboard.putNumber("I Gain", kI);
//     SmartDashboard.putNumber("D Gain", kD);
//     SmartDashboard.putNumber("I Zone", kIz);
//     SmartDashboard.putNumber("Feed Forward", kFF);
//     SmartDashboard.putNumber("Max Output", kMaxOutput);
//     SmartDashboard.putNumber("Min Output", kMinOutput);
//     SmartDashboard.putNumber("Set Rotations", 0);

    // SmartDashboard.putData("mech2d", mechanism2d);
//   }

//   @Override
//   public void robotPeriodic() {
//   }

//   /** This function is called periodically during operator control. */
//   @Override
//   public void teleopPeriodic() {
//     double p = SmartDashboard.getNumber("P Gain", 0);
//     double i = SmartDashboard.getNumber("I Gain", 0);
//     double d = SmartDashboard.getNumber("D Gain", 0);
//     double iz = SmartDashboard.getNumber("I Zone", 0);
//     double ff = SmartDashboard.getNumber("Feed Forward", 0);
//     double max = SmartDashboard.getNumber("Max Output", 0);
//     double min = SmartDashboard.getNumber("Min Output", 0);
//     double rotations = SmartDashboard.getNumber("Set Rotations", 0);

//     // if PID coefficients on SmartDashboard have changed, write new values to
//     // controller
//     if ((p != kP)) {
//       pidController.setP(p);
//       kP = p;
//     }
//     if ((i != kI)) {
//       pidController.setI(i);
//       kI = i;
//     }
//     if ((d != kD)) {
//       pidController.setD(d);
//       kD = d;
//     }
//     if ((iz != kIz)) {
//       pidController.setIZone(iz);
//       kIz = iz;
//     }
//     if ((ff != kFF)) {
//       pidController.setFF(ff);
//       kFF = ff;
//     }
//     if ((max != kMaxOutput) || (min != kMinOutput)) {
//       pidController.setOutputRange(min, max);
//       kMinOutput = min;
//       kMaxOutput = max;
//     }

//     pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);

//     SmartDashboard.putNumber("ProcessVariable", absoluteEncoder.getPosition());
//   }

//   /** This function is called once when the robot is disabled. */
//   @Override
//   public void disabledInit() {
//   }

//   /** This function is called periodically when disabled. */
//   @Override
//   public void disabledPeriodic() {
//   }

//   /** This function is called once when test mode is enabled. */
//   @Override
//   public void testInit() {
//   }

//   /** This function is called periodically during test mode. */
//   @Override
//   public void testPeriodic() {
//   }

//   /** This function is called once when the robot is first started up. */
  // @Override
  // public void simulationInit() {
  //   REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
  // }

  // /** This function is called periodically whilst in simulation. */
  // @Override
  // public void simulationPeriodic() {
  //   REVPhysicsSim.getInstance().run();

  //   square.setAngle(new Rotation2d(Units.rotationsToDegrees(absoluteEncoder.getPosition())));


  // }
// }
