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

@TeleOp(name="TTDriveB", group="Linear Opmode")
public class TTDriveB extends LinearOpMode {

    //---------------Declare Hardware Variables-----------------------//
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private Servo airplane = null;
    private Servo armROT = null;
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
    double ClosedLeft = 0;
    double ClosedRight = 0.2;
    double FrontScoreClaw = 0.075;

    double MidHangingArm = 0.255;
    double FrontScoreArm = 0.17;
    double HangArm = 0.27;
    double BackScoreArm = 0.31;
double BackScoreClaw = 0.62;
    double OpenLeft = 0.2;
    double OpenRight = 0;
    double GroundClaw = 0.025;

    double GroundArm = 0.095;
    int TapeLVL = 2;
    boolean LClosed = true;
    boolean RClosed = true;

    //---------------Run OpMode-----------------------------//
    @Override
    public void runOpMode() {
        //---------------Init Hardware-----------------------//
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        arm = hardwareMap.get(DcMotor.class, "arm");
        airplane = hardwareMap.get(Servo.class, "airplane");
        armROT = hardwareMap.get(Servo.class,"armROT");
        clawleft = hardwareMap.get(Servo.class, "clawleft");
        clawright = hardwareMap.get(Servo.class, "clawright");
        clawrotate = hardwareMap.get(Servo.class, "clawrotate");
        //---------------Setup Motors-----------------------//
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //---------------Setup Servos-----------------------//
        armR = GroundArm;
        clawright.setPosition(ClosedRight);
        clawleft.setPosition(ClosedLeft);
        clawrotate.setPosition(GroundClaw);
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
            double axial   = -gamepad1.left_stick_y;  //Pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0)
            {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            //----------Arm-----------//

            if(gamepad1.b)
            {
                arm.setPower(1);
            }
            else
            {
                arm.setPower(0);
            }

            if(gamepad1.a)
            {
                arm.setPower(-1);
            }
            else
            {
                arm.setPower(0);
            }

            //------Arm Rotate--------//
            armROT.setPosition(armR);









            //---------Airplane----------//

            if(gamepad1.y)
            {
                airplane.setPosition(0.5);
            }
            else
            {
                airplane.setPosition(0.1);
            }

            //----------Claws & Claw Rotate----------//

            if(gamepad1.right_trigger > 0.5)
            {
                /*if(RClosed == true){
                    clawright.setPosition(OpenRight);
                    RClosed = false;
                    sleep(400);
                }
                if(RClosed == false){
                    clawright.setPosition(ClosedRight);
                    RClosed = true;
                    sleep(400);
                }
                */
                 clawright.setPosition(ClosedRight);
                 clawleft.setPosition(ClosedLeft);

            }

            if(gamepad1.left_trigger > 0.5)
            {
              /*  if(LClosed == true){
                    clawleft.setPosition(OpenLeft);
                    LClosed = false;
                }
                if(LClosed == false){
                    clawleft.setPosition(ClosedLeft);
                    RClosed = true;
                }

               */
                clawleft.setPosition(OpenLeft);
                clawright.setPosition(OpenRight);
            }

            if(gamepad1.x)
            {
                clawleft.setPosition(ClosedLeft);
                clawright.setPosition(ClosedRight);
            }

           /*

            if(gamepad2.dpad_left)
            {
                armR = 0.27;
                sleep(500);
                clawrotate.setPosition(0);
            }

            if(gamepad2.dpad_right)
            {
                armR = 0.255;
            }

            */

            //--------------Arm-Presets---------------//
            if(gamepad1.right_stick_button)
            {

                clawrotate.setPosition(GroundClaw);
                armR = GroundArm;
                clawright.setPosition(ClosedRight);
                clawleft.setPosition(ClosedLeft);


            }

            if(gamepad1.left_stick_button)
            {
                clawright.setPosition(ClosedRight);
                clawleft.setPosition(ClosedLeft);
                if(TapeLVL == 1){
                    clawrotate.setPosition(FrontScoreClaw);
                }
                if(TapeLVL == 2){
                    clawrotate.setPosition(BackScoreClaw);
                }

                sleep(100);
                if(TapeLVL == 1){
                    armR = FrontScoreArm;
                }
                if(TapeLVL == 2){
                    armR = BackScoreArm;
                }

            }

            //-----------Speed Control------------//

            if(gamepad1.left_bumper)
            {
                bspeed = 1;
            }

            if(gamepad1.right_bumper)
            {
                bspeed = 2;
            }

            if(bspeed == 1)
            {
                lfspeed = leftFrontPower/2;
                rfspeed = rightFrontPower/2;
                lbspeed = leftBackPower/2;
                rbspeed = rightBackPower/2;
            }

            if(bspeed == 2)
            {
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

        }}}







