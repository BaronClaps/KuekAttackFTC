package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TTDriveCode", group="Linear Opmode")
public class TTDriveCode extends LinearOpMode {

    //---------------Declare Hardware Variables-----------------------//
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private Servo airplane = null;
    private DcMotor gearROT = null;
    private Servo clawleft = null;
    private Servo clawright = null;
    private Servo clawrotate = null;

    //---------------Declare Variables-----------------------//
    private int bspeed;
    private double rfspeed;
    private double lfspeed;
    private double rbspeed;
    private double lbspeed;
    private double armR;
    //---------------Declare Servo Variables-----------------//
    double ClosedLeft = 0.03;
    double ClosedRight = 0.145;
    double OpenLeft = 0.175;
    double OpenRight = 0;
    double GroundClaw = 0.1175;
    double ScoringClaw = 0.7;

    //---------------Run OpMode-----------------------------//
    @Override
    public void runOpMode() {
        //---------------Init Hardware-----------------------//
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        arm = hardwareMap.get(DcMotor.class, "arm");
        airplane = hardwareMap.get(Servo.class, "airplane");
        gearROT = hardwareMap.get(DcMotor.class, "gearROT");
        clawleft = hardwareMap.get(Servo.class, "clawleft");
        clawright = hardwareMap.get(Servo.class, "clawright");
        clawrotate = hardwareMap.get(Servo.class, "clawrotate");
        //---------------Setup Motors-----------------------//
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        gearROT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gearROT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(0);
        gearROT.setTargetPosition(13);
        //---------------Setup Servos-----------------------//
        clawright.setPosition(ClosedRight);
        clawleft.setPosition(ClosedLeft);
        bspeed = 2;
        //---------------Wait until Play-----------------------//
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        //---------------Loop while Active-----------------------//
        while (opModeIsActive()) {

            //--------Joysticks Controls & Wheel Power-----------//
            double max;
            double axial = -gamepad1.left_stick_y;  //Pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            //-----------------Arm Rotate & Arm Preset-----------------//

            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gearROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad2.dpad_up)
            {
                raeg(1);
            }

            if (gamepad2.dpad_down)
            {
                raeg(-1);
            }

            if (gamepad2.a)
            {
              tfil(-1);
            }

            if (gamepad2.b)
            {
                tfil(1);
            }

            //Must be pressed before start of match
            if (gamepad1.x)
            {
                gearROT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad2.x){
                clawrotate.setPosition(GroundClaw);
            }

            //---------Airplane----------//

            if (gamepad1.y) {
                airplane.setPosition(0.5);
            } else {
                airplane.setPosition(0.1);
            }

            //----------Claws & Claw Rotate----------//

            if (gamepad2.right_trigger > 0.5) {
                clawleft.setPosition(ClosedLeft);
                clawright.setPosition(ClosedRight);
                sleep(200);
                raegPosition(50,0.33);
                clawrotate.setPosition(ScoringClaw);

            }

            if (gamepad2.left_trigger > 0.5) {
                clawright.setPosition(OpenRight);
                clawleft.setPosition(OpenLeft);
            }
            if(gamepad2.right_bumper){
                clawright.setPosition(ClosedRight);
                clawleft.setPosition(ClosedLeft);
            }

            //Below are weird due to manual control whilst we score backwards
            if(gamepad2.dpad_right) {
                clawleft.setPosition(OpenLeft);
            }
            if(gamepad2.dpad_left) {
                clawright.setPosition(OpenRight);
            }

            //--------------Arm-Presets---------------//

            if(gamepad2.right_stick_button){
                clawright.setPosition(ClosedRight);
                clawleft.setPosition(ClosedLeft);
                clawrotate.setPosition(GroundClaw);
                tfilPosition(0,1);
                sleep(400);
                raegPosition(0, 0.125);
            }

            if(gamepad2.left_stick_button){
                clawright.setPosition(ClosedRight);
                clawleft.setPosition(ClosedLeft);
                clawrotate.setPosition(ScoringClaw);
                sleep(400);
                raegPosition(630, 0.33);
            }

            //-----------Speed Control------------//

            if (gamepad1.left_bumper) {
                bspeed = 1;
            }

            if (gamepad1.right_bumper) {
                bspeed = 2;
            }

            if (bspeed == 1) {
                lfspeed = leftFrontPower / 2;
                rfspeed = rightFrontPower / 2;
                lbspeed = leftBackPower / 2;
                rbspeed = rightBackPower / 2;
            }

            if (bspeed == 2) {
                lfspeed = leftFrontPower;
                rfspeed = rightFrontPower;
                lbspeed = leftBackPower;
                rbspeed = rightBackPower;
            }

            leftFrontDrive.setPower(lfspeed);
            rightFrontDrive.setPower(rfspeed);
            leftBackDrive.setPower(lbspeed);
            rightBackDrive.setPower(rbspeed);

            //-------------Display Timer & Wheel Power---------------//
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
    public void raegPosition (int h, double H) {gearROT.setTargetPosition(h); gearROT.setPower(H); }
    public void tfilPosition (int e, double E) {arm.setTargetPosition(e); arm.setPower(E); }
    public void raeg (int s) {
        gearROT.setPower(0.333);
        gearROT.setTargetPosition(gearROT.getCurrentPosition() + 50 * s);
    }
    public void tfil (int s) {
        arm.setPower(1);
        arm.setTargetPosition(arm.getCurrentPosition() + 100 * s);
    }
    public void fasttfil (int s) {
        arm.setPower(1);
        arm.setTargetPosition(arm.getCurrentPosition() + 250 * s);
        if (arm.getTargetPosition() > 2500) {//sets a limit on the lift distance
            arm.setTargetPosition(2500); }
    }

}