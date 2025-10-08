package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@TeleOp
public class PIDF_Arm extends OpMode {

  public PIDController controller;

  /* Here we name all the motors that we are using that aren't the wheel motors with the format of pid
     Will probably only really use pidf for the elbow system but im adding the rest just in case */

  public PIDController rsPID, lsPID, beltPID, elbowPID;

  // Our pidf values for the launching motors
  public static double rsP = 0, rsI = 0, rsD = 0, rsF = 0;

  // Our pidf values for the intake
  public static double beltP = 0, beltI = 0, beltD = 0, beltF = 0;

  // Our pidf values for the elbow joint motor
  public static double elbowP = 0, elbowI = 0, elbowD = 0;
  public static double  elbowF = 0;

  // Declaring a target position for motors to go to
  public static double rsTarget;
  public static double lsTarget;
  public static double beltTarget;
  public static double elbowTarget;


  // Declaring a double as a target position for our motors, could have it return an int but not too important
  public void setRsTarget(double b) {
    rsTarget = b;
  }

  public void setLsTarget(double b) {
    lsTarget = b;
  }

  public void setBeltTarget(double b) {
    beltTarget = b;
  }

  public void setElbowTarget(double b) {
    elbowTarget = b;
  }

  /* Now we have to make a variable for each of the values depending on the different motor rpm
     The 6000 rpms set as rsPID and lsPID is 28 PPR
     The elbowPID is unknown rn
     The 435 rpm motor beltPID is 384.5  PPR  */

  private final double rsTicks_In_Degrees = 28; // encoder resolution for the 6000 rpm motor
  private final double lsTicks_In_Degrees = 28;
  private final double beltTicks_In_Degrees = 384.5; // encoder resolution for the 435 rpm motor

  private final double elbowTicks_In_Degrees = 0;   // CHANGE THIS ONCE MOTOR IS ATTACHTECDDDDD PLSLSLSLS

  private DcMotor rs;
  private DcMotor ls;
  private DcMotor belt;
  private DcMotor elbow;


  @Override
  public void init() {
//    START HERE TMRW
//    controller = new PIDController(p,i,d);
//    rsPID = new PIDController(p,i,d);

    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

  }


  @Override
  public void loop() {

  }

}



