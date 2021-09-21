/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="AdminMode", group="Linear Opmode")
//@Disabled
public class AdminMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightRear = null;
    private DcMotor rightFront = null;
    private DcMotor launcherMotor = null;
    private Servo wobbleGrabber = null;
    private DcMotor rampDrive1 = null;
    private DcMotor rampDrive2 = null;
    private DcMotor collectionMotor = null;
    private Servo grabberArm  =null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        launcherMotor = hardwareMap.get(DcMotor.class, "launcher_motor");
        wobbleGrabber = hardwareMap.get(Servo.class,"wobble_grabber");
        grabberArm = hardwareMap.get(Servo.class,"grabber_Arm");
        rampDrive1 = hardwareMap.get(DcMotor.class,"ramp_drive1");
        rampDrive2 = hardwareMap.get(DcMotor.class,"ramp_drive2");
        collectionMotor = hardwareMap.get(DcMotor.class, "collection_motor");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        //rampDrive1.setDirection(DcMotor.Direction.REVERSE);
        //rampDrive2.setDirection(DcMotor.Direction.REVERSE);
        collectionMotor.setDirection(DcMotor.Direction.REVERSE);

/*
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        */


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double speedLimiter = 1.0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_y,gamepad1.left_stick_x);

            double robotAngle = Math.atan2(gamepad1.left_stick_y,gamepad1.left_stick_x)- Math.PI / 4;

            double v1 = r * Math.cos(robotAngle);

            double v2 = r * Math.sin(robotAngle);

            double v3 = r * Math.sin(robotAngle);

            double v4 = r * Math.cos(robotAngle);

            /*
            v1  = Range.clip(v1, -1.0, 1.0) ;
            v2  = Range.clip(v2, -1.0, -1.0) ;
            v3  = Range.clip(v3, 1.0, 1.0) ;
            v4  = Range.clip(v4, 1.0, -1.0) ;
            */

            leftFront.setPower(((-v1 + gamepad1.right_stick_x)*speedLimiter));
            leftRear.setPower(((-v2 + gamepad1.right_stick_x)*speedLimiter));
            rightFront.setPower(((-v3 - gamepad1.right_stick_x)*speedLimiter));
            rightRear.setPower(((-v4 - gamepad1.right_stick_x)*speedLimiter));



            if(gamepad1.right_trigger>0)
            {
            launcherMotor.setPower(.8); //normally.8
            }
            if(gamepad1.left_trigger>0)
            {
            launcherMotor.setPower(0);
            }


            if(gamepad1.dpad_up)
            {
                rampDrive1.setPower(1);
                rampDrive2.setPower(1);
                collectionMotor.setPower(1);
            }
            if(gamepad1.dpad_down)
            {
                rampDrive1.setPower(-1);
                rampDrive2.setPower(-1);
                collectionMotor.setPower(-1);
            }
            else
            {
                rampDrive1.setPower(0);
                rampDrive2.setPower(0);
                collectionMotor.setPower(0);
            }


            if(gamepad1.left_bumper)
            {
                wobbleGrabber.setPosition(1);
            }
            else
            {
                wobbleGrabber.setPosition(0);
            }


            if(gamepad1.y)
            {
                grabberArm.setPosition(0.5);
            }

            if(gamepad1.b)
            {
                grabberArm.setPosition(0);
            }

            if(gamepad1.x)
            {
                grabberArm.setPosition(1);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("LF Power", leftFront.getPower());
            telemetry.addData("LF Position", leftFront.getCurrentPosition());
            telemetry.addData("RF Power", rightFront.getPower());
            telemetry.addData("RF Position", rightFront.getCurrentPosition());
            telemetry.addData("LR Power", leftRear.getPower());
            telemetry.addData("LR Position", leftRear.getCurrentPosition());
            telemetry.addData("RR Power", rightRear.getPower());
            telemetry.addData("RR Position", rightRear.getCurrentPosition());
            telemetry.addData("Launcher", launcherMotor.getPower());
            telemetry.addData("Ramp Drive1", rampDrive1.getPower());
            telemetry.addData("Ramp Drive2", rampDrive2.getPower());
            telemetry.addData("Grabber Position", wobbleGrabber.getPosition());
            telemetry.update();
        }
    }
}
