package teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
  public class PIDF_Arm extends OpMode {
    
    public PIDController controller;

    // Here we name all the motors that we are using that aren't the wheel motors with the format of pid
    public PIDController rsPID, lsPID, beltPID, elbowPID;

    // Now we have to make a variable for each of the values depending on the different motor rpm
    // The 6000 rpms set as rsPID and lsPID
    // The elbowPID is unknown rn
    // The beltPID is unknown rn

  }
