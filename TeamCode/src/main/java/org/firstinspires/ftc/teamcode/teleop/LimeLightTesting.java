package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;


    @TeleOp
    public class LimeLightTesting extends LinearOpMode {
        // CAMERA VARS

        private Limelight3A cam;


        @Override
        public void runOpMode() {

            cam = hardwareMap.get(Limelight3A.class, "cam");

            // No clue what this does atm
            telemetry.setMsTransmissionInterval(11);

            // Ensures we are in pipeline 0 which has been tuned for AprilTags
            cam.pipelineSwitch(0);

            // 50 hz should be more than good for anything we'll need
            cam.setPollRateHz(20);

            // Starts the cam
            cam.start();


            Pose2D pose;

            waitForStart();

                while (opModeIsActive()) {
                    // Printing things to station, snatched this off the ll website
                    LLStatus status = cam.getStatus();
                    telemetry.addData("Name", "%s", status.getName());
                    telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(),(int)status.getFps());
                    telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());

                    LLResult result = cam.getLatestResult();
                    if (result.isValid()) {
                        // Some general info that I also yoinked from the ll website
                        Pose3D botpose = result.getBotpose();

                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("txnc", result.getTxNC());
                        telemetry.addData("ty", result.getTy());
                        telemetry.addData("tync", result.getTyNC());

                        telemetry.addData("Botpose", botpose.toString());


                        // This list should have the april tags info alr on it
                        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                        for (LLResultTypes.FiducialResult fr : fiducialResults) {
                            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                        }
                    } else {
                        telemetry.addData("Limelight", "No data available");

                    }

                    telemetry.update();
                }
                cam.stop();
            }
        }

