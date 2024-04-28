package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name="TTRoadRed")

public class TTRoadRed extends LinearOpMode{

    private final int READ_PERIOD = 1;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private Servo clawrotate = null; //es1
    private Servo clawleft = null; //es1
    private Servo clawright = null; //es2
    private DcMotor gearROT = null;

    //---------------Declare Variables-----------------------//
    private double armR;
    //---------------Declare Servo Variables-----------------//
    double ClosedLeft = 0.03;
    double ClosedRight = 0.145;
    double OpenLeft = 0.175;
    double OpenRight = 0;
    double GroundClaw = 0.1175;
    double ScoringClaw = 0.7;
    private HuskyLens huskyLens;
    //----------------April Tag Detection Values--------------//
    double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    //-------------------Camera Initialization---------------------------//
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override public void runOpMode() {
        Pose2d beginPose = new Pose2d(60, 12, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive1           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        arm = hardwareMap.get(DcMotor.class, "arm");
        clawrotate = hardwareMap.get(Servo.class, "clawrotate");
        clawleft = hardwareMap.get(Servo.class, "clawleft");
        clawright = hardwareMap.get(Servo.class, "clawright");
        gearROT = hardwareMap.get(DcMotor.class, "gearROT");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        //---------------Setup Motors-----------------------//
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        gearROT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gearROT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawright.setPosition(ClosedRight);
        clawleft.setPosition(ClosedLeft);

        // Use low exposure time to reduce motion blur
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }//makes sure the huskylens is talking to the control hub
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);// can change to other algorithms


        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // Drive to target Automatically
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive1  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();// from huskylens
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());// this gives you the data
                telemetry.addData("location?", blocks[i].x);// this gives you just x
                //TODO ensure your x values of the husky lens are appropriate to the desired areas

                //----------------------------1----------------------------\\
                if (blocks[i].x < 90 && blocks[i].id == 1) {
                    Actions.runBlocking(
                            drive.actionBuilder(beginPose)
                                    .stopAndAdd(StartPos())
                                    .splineTo(new Vector2d(35,11),-1*Math.PI/2)
                                    .waitSeconds(.5)
                                    .stopAndAdd(openR())
                                    .waitSeconds(0.5)
                                    .stopAndAdd(scoringPos1())
                                    .waitSeconds(.5)
                                    .stopAndAdd(liftExtend2())
                                    .waitSeconds(.5)
                                    .strafeTo(new Vector2d(22.5, 48))
                                    .waitSeconds(0.5)
                                    .stopAndAdd(moveRobot(drive1, strafe, turn))
                                    .waitSeconds(0.5)
                                    .stopAndAdd(openL())
                                    .waitSeconds(.5)
                                    .lineToY(40)
                                    .stopAndAdd(liftIn2())
                                    .waitSeconds(.5)
                                    .stopAndAdd(closeL())
                                    .stopAndAdd(closeR())
                                    .strafeTo((new Vector2d(66, 54)))
                                    .stopAndAdd(geardownTEST())
                                    .waitSeconds(.5)
                                    .stopAndAdd(GearROT0())
                                    .build());
                    sleep(400000);
                }

                //----------------------------2----------------------------\\
                if (blocks[i].x > 90 && blocks[i].x < 180 && blocks[i].id == 1) {
                    Actions.runBlocking(
                            drive.actionBuilder(beginPose)
                                    .stopAndAdd(StartPos())
                                    .strafeTo(new Vector2d(39.5, 13))
                                    .stopAndAdd(openR())
                                    .waitSeconds(.5)
                                    .lineToX(40)
                                    .turn(PI / 2)
                                    .stopAndAdd(scoringPos1())
                                    .stopAndAdd(closeR())
                                    .stopAndAdd(liftExtend2())
                                    .waitSeconds(.5)
                                    .strafeTo(new Vector2d(27, 48))
                                    .waitSeconds(.5)
                                    .stopAndAdd(moveRobot(drive1, strafe, turn))
                                    .waitSeconds(0.5)
                                    .stopAndAdd(openL())
                                    .waitSeconds(.5)
                                    .lineToY(40)
                                    .stopAndAdd(closeL())
                                    .stopAndAdd(liftIn2())
                                    .strafeTo((new Vector2d(60.5, 50)))
                                    .stopAndAdd(geardownTEST())
                                    .waitSeconds(.5)
                                    .stopAndAdd(GearROT0())
                                    .lineToY(56)
                                    .build());
                    sleep(400000);
                }
                //----------------------------3----------------------------\\
                if (blocks[i].x > 180 && blocks[i].id == 1) {
                    Actions.runBlocking(
                            drive.actionBuilder(beginPose)
                                    .stopAndAdd(StartPos())
                                    .strafeTo(new Vector2d(29.5, 40))
                                    .waitSeconds(.5)
                                    .turn(PI / 2)
                                    .stopAndAdd(openR())
                                    .waitSeconds(.1)
                                    .stopAndAdd(scoringPos())
                                    .strafeTo(new Vector2d(43.5, 48))
                                    .stopAndAdd(closeR())
                                    .stopAndAdd(liftExtend2())
                                    .waitSeconds(.25) //test this value
                                    .lineToY(55)
                                    .waitSeconds(0.5)
                                    .stopAndAdd(moveRobot(drive1, strafe, turn))
                                    .waitSeconds(0.5)
                                    .stopAndAdd(openL())
                                    .waitSeconds(.1)
                                    .lineToY(42)
                                    .strafeTo((new Vector2d(67.5, 50)))
                                    .stopAndAdd(liftIn2())
                                    .stopAndAdd(closeL())
                                    .waitSeconds(.5)
                                    .stopAndAdd(geardownTEST())
                                    .waitSeconds(.5)
                                    .stopAndAdd(GearROT0())
                                    .lineToY(63)
                                    .build());
                    sleep(400000);


                }


            }
        }
    }

    public void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "webcam1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public Action moveRobot(double x, double y, double yaw){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double leftFrontPower    =  x -y -yaw;
                double rightFrontPower   =  x +y +yaw;
                double leftBackPower     =  x +y -yaw;
                double rightBackPower    =  x -y +yaw;

                // Normalize wheel powers to be less than 1.0
                double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                // Send powers to the wheels.
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
                return false;
            }
        };
    }

    public Action liftExtend(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(-600);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                return false;
            }
        };
    }

    public Action geardown(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawrotate.setPosition(GroundClaw);
                gearROT.setTargetPosition(-800);
                gearROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gearROT.setPower(0.5);
                gearROT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gearROT.setPower(0);
                return false;
            }
        };
    }
    public Action geardownTEST(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawrotate.setPosition(GroundClaw);
                gearROT.setTargetPosition(200);
                gearROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gearROT.setPower(0.4);

                return false;
            }
        };
    }
    public Action StartPos(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                gearROT.setTargetPosition(75);
                gearROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gearROT.setPower(0.4);
                clawrotate.setPosition(GroundClaw);
                return false;
            }
        };
    }
    public Action GearROT0(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                gearROT.setPower(0);
                return false;
            }
        };
    }
    public Action liftIn(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(600);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                return false;
            }
        };
    }

    public Action liftExtend2(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(-200);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.7);
                return false;
            }
        };
    }
    public Action liftIn2(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(200);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.7);
                return false;
            }
        };
    }

    public Action groundclaw(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawrotate.setPosition(GroundClaw);
                return false;
            }
        };
    }

    public Action openR(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawright.setPosition(OpenRight);
                return false;
            }
        };
    }

    public Action openL(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawleft.setPosition(OpenLeft);
                return false;
            }
        };
    }

    public Action closeR(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawright.setPosition(ClosedRight);
                return false;
            }
        };
    }

    public Action closeL(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawleft.setPosition(ClosedLeft);
                return false;
            }
        };
    }

    public Action scoringPos(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawright.setPosition(ClosedRight);
                clawrotate.setPosition(ScoringClaw);
                gearROT.setTargetPosition(630);
                gearROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gearROT.setPower(0.4);
                return false;
            }
        };
    }
    public Action scoringPos1(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawrotate.setPosition(ScoringClaw);
                gearROT.setTargetPosition(670);
                gearROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gearROT.setPower(0.4);
                return false;
            }
        };
    }

//    public Action downposition(){
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                clawrotate.setPosition(GroundClaw);
//                gearROT.setTargetPosition();
//                gearROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                gearROT.setPower(0.333);
//                gearROT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                gearROT.setPower(0);
//                return false;
//            }
//        };
//    }




}