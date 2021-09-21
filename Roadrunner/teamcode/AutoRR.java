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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


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

@Autonomous(name="AutoRR", group="A")
//@Disabled
public class AutoRR extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightRear = null;
    private DcMotor rightFront = null;
    private DcMotorEx launcherMotor = null;
    private Servo wobbleGrabber = null;
    private DcMotor rampDrive1 = null;
    private DcMotor rampDrive2 = null;
    private DcMotor collectionMotor = null;
    private Servo grabberArm = null;
    private CRServo knockydowny =null;

    public void stopRampDrive()
    {
        rampDrive1.setPower(0);
        rampDrive2.setPower(0);
    }

    public void forwardRampDrive()
    {
        rampDrive1.setPower(1);
        rampDrive2.setPower(1);
    }

    public void backwardRampDrive()
    {
        rampDrive1.setPower(-1);
        rampDrive2.setPower(-1);
    }

    //Ring determination program
    OpenCvWebcam webcam;
    RingDeterminationPipelinev2 pipeline = new RingDeterminationPipelinev2();

    @Override
    public void runOpMode() {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT);
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.update();
        // Don't burn CPU cycles busy-looping in this sample
        sleep(50);

        telemetry.addData("Status: ", "Initializing Hardware");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        wobbleGrabber = hardwareMap.get(Servo.class, "wobble_grabber");
        grabberArm = hardwareMap.get(Servo.class, "grabber_Arm");
        rampDrive1 = hardwareMap.get(DcMotor.class, "ramp_drive1");
        rampDrive2 = hardwareMap.get(DcMotor.class, "ramp_drive2");
        collectionMotor = hardwareMap.get(DcMotor.class, "collection_motor");
        knockydowny = hardwareMap.get(CRServo.class, "knocky_downy");

        rampDrive1.setDirection(DcMotor.Direction.REVERSE);
        //rampDrive2.setDirection(DcMotor.Direction.REVERSE);
        collectionMotor.setDirection(DcMotor.Direction.REVERSE);




        //start the robot at x= -60, y= -45, heading= 0
        Pose2d startPose = new Pose2d(-60,-45, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        //Four_________________________________________________________________________________________________________________________________________________
        //Drive to far box
        Trajectory fourStep1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading( new Pose2d(52,-60,Math.toRadians(280)))
                .build();

        //Drive to middle
        Trajectory fourStep2 = drive.trajectoryBuilder(fourStep1.end())
                .lineToLinearHeading(new Pose2d(-10,-15,Math.toRadians(90)))
                .build();

        //Left to 2nd wobble
        Trajectory fourStep3 = drive.trajectoryBuilder(fourStep2.end())
                .strafeLeft(19)
                .build();

        //right back to middle
        Trajectory fourStep4 = drive.trajectoryBuilder(fourStep3.end())
                .strafeRight(19)
                .build();

        //drive back to far box
        Trajectory fourStep5 = drive.trajectoryBuilder(fourStep4.end())
                .lineToLinearHeading( new Pose2d(42,-65,Math.toRadians(-110)))
                .build();

        //drive to powershot position
        Trajectory fourStep6 = drive.trajectoryBuilder(fourStep5.end())
                .lineToLinearHeading(new Pose2d(-5,-5,Math.toRadians(0)))
                .build();

        //park
        Trajectory fourStep7 = drive.trajectoryBuilder(fourStep6.end())
                .forward(15)
                .build();

        //One_________________________________________________________________________________________________________________________________________________
        //Drive to middle box
        Trajectory oneStep1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(35,-45, Math.toRadians(-40)))
                .build();

        //Back away from wobble
        Trajectory oneStep2 = drive.trajectoryBuilder(oneStep1.end())
                .strafeRight(10)
                .build();

        //Drive to middle
        Trajectory oneStep3 = drive.trajectoryBuilder(oneStep2.end())
                .lineToLinearHeading(new Pose2d(-10,-15,Math.toRadians(90)))
                .build();

        //Left to 2nd wobble
        Trajectory oneStep4 = drive.trajectoryBuilder(oneStep3.end())
                .strafeLeft(19)
                .build();

        //right back to middle
        Trajectory oneStep5 = drive.trajectoryBuilder(oneStep4.end())
                .strafeRight(15)
                .build();

        //drive back to middle box
        Trajectory oneStep6 = drive.trajectoryBuilder(oneStep5.end())
                .lineToLinearHeading( new Pose2d(25,-50,Math.toRadians(-70)))
                .build();

        //drive to powershot position
        Trajectory oneStep7 = drive.trajectoryBuilder(oneStep6.end())
                .lineToLinearHeading(new Pose2d(-5,-5,Math.toRadians(0)))
                .build();

        //park
        Trajectory oneStep8 = drive.trajectoryBuilder(oneStep7.end())
                .forward(17)
                .build();

        //None_________________________________________________________________________________________________________________________________________________
        Trajectory noneStep1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading( new Pose2d(3,-65,Math.toRadians(270)))
                .build();

        Trajectory noneStep2 = drive.trajectoryBuilder(noneStep1.end())
                .lineToLinearHeading(new Pose2d(-10,-15,Math.toRadians(90)))
                .build();

        //Left to 2nd wobble
        Trajectory noneStep3 = drive.trajectoryBuilder(noneStep2.end())
                .strafeLeft(19
                )
                .build();

        //
        Trajectory noneStep4 = drive.trajectoryBuilder(noneStep3.end())
                .lineToLinearHeading( new Pose2d(-7,-65,Math.toRadians(270)))
                .build();

        Trajectory noneStep5 = drive.trajectoryBuilder(noneStep4.end())
                .strafeRight(5)
                .build();

        //drive to powershot position
        Trajectory noneStep6 = drive.trajectoryBuilder(noneStep5.end())
                .lineToLinearHeading(new Pose2d(-5,-5,Math.toRadians(0)))
                .build();

        //park
        Trajectory noneStep7 = drive.trajectoryBuilder(noneStep6.end())
                .forward(17)
                .build();

        telemetry.addData("Init:", "Complete");

        waitForStart();
        runtime.reset();

        //beginning of Autonomous Program
        sleep(400);
        int avg1 = pipeline.getAnalysis();
        telemetry.addData("Average:", pipeline.getAnalysis());
        telemetry.update();
        sleep(400);
        telemetry.addData("Average:", pipeline.getAnalysis());
        telemetry.update();
        sleep(400);

        knockydowny.setPower(.6);

        //_________________________________________________________________________________________________________________________________________________

        if(avg1 >141)//far Zone
        {

            grabberArm.setPosition(1);
            wobbleGrabber.setPosition(0);

            drive.followTrajectory(fourStep1);
            grabberArm.setPosition(0);
            sleep(600);
            wobbleGrabber.setPosition(1);
            sleep(800);
            grabberArm.setPosition(1);
            sleep(500);

            drive.followTrajectory(fourStep2);
            grabberArm.setPosition(0);

            drive.followTrajectory(fourStep3);

            wobbleGrabber.setPosition(0);
            sleep(700);
            grabberArm.setPosition(0.5);


            drive.followTrajectory(fourStep4);
            drive.followTrajectory(fourStep5);
            grabberArm.setPosition(0);
            sleep(450);
            wobbleGrabber.setPosition(1);
            sleep(800);
            grabberArm.setPosition(1);


            drive.followTrajectory(fourStep6);
            drive.turn(Math.toRadians(-7));

            //fire 1st ring at power shot
            backwardRampDrive();
            sleep(100);
            stopRampDrive();
            launcherMotor.setVelocity(1500);
            sleep(1300);
            forwardRampDrive();
            sleep(600);
            stopRampDrive();
            sleep(200);

            drive.turn(Math.toRadians(7));

            //fire 2nd ring at power shot
            launcherMotor.setVelocity(1650);
            forwardRampDrive();
            sleep(750);
            stopRampDrive();
            sleep(400);

            drive.turn(Math.toRadians(7));

            //fire 3rd ring at power shot
            launcherMotor.setVelocity(1650);

            forwardRampDrive();
            sleep(1200);
            stopRampDrive();
            launcherMotor.setPower(0);
            sleep(400);

            drive.followTrajectory(fourStep7);
            grabberArm.setPosition(0.5);
            knockydowny.setPower(-.6);
            sleep(1000);
            knockydowny.setPower(0);


        }
        else if (avg1 > 131)// middle Zone
        {
            grabberArm.setPosition(1);
            wobbleGrabber.setPosition(0);

            drive.followTrajectory(oneStep1);
            grabberArm.setPosition(0);
            sleep(600);
            wobbleGrabber.setPosition(1);
            sleep(800);
            grabberArm.setPosition(1);
            sleep(500);

            drive.followTrajectory(oneStep2);
            grabberArm.setPosition(0);
            drive.followTrajectory(oneStep3);
            drive.followTrajectory(oneStep4);


            grabberArm.setPosition(0);
            wobbleGrabber.setPosition(0);
            sleep(600);
            grabberArm.setPosition(0.5);


            drive.followTrajectory(oneStep5);
            drive.followTrajectory(oneStep6);
            grabberArm.setPosition(0);
            sleep(450);
            wobbleGrabber.setPosition(1);
            sleep(800);
            grabberArm.setPosition(1);


            drive.followTrajectory(oneStep7);
            drive.turn(Math.toRadians(-5));

            //fire 1st ring at power shot
            backwardRampDrive();
            sleep(100);
            stopRampDrive();
            launcherMotor.setVelocity(1500);
            sleep(1500);
            forwardRampDrive();
            sleep(600);
            stopRampDrive();
            sleep(200);

            drive.turn(Math.toRadians(6));

            //fire 2nd ring at power shot
            launcherMotor.setVelocity(1650);
            forwardRampDrive();
            sleep(750);
            stopRampDrive();
            sleep(400);

            drive.turn(Math.toRadians(7));

            //fire 3rd ring at power shot
            launcherMotor.setVelocity(1650);
            forwardRampDrive();
            sleep(1200);
            stopRampDrive();
            launcherMotor.setPower(0);
            sleep(400);

            drive.followTrajectory(oneStep8);
            grabberArm.setPosition(0.5);
            knockydowny.setPower(-.6);
            sleep(1000);
            knockydowny.setPower(0);

        }
        else //close Zone
            {
                grabberArm.setPosition(1);
                wobbleGrabber.setPosition(0);

                drive.followTrajectory(noneStep1);
                grabberArm.setPosition(0);
                sleep(600);
                wobbleGrabber.setPosition(1);
                sleep(800);
                grabberArm.setPosition(1);
                sleep(500);


                drive.followTrajectory(noneStep2);
                grabberArm.setPosition(0);

                drive.followTrajectory(noneStep3);
                grabberArm.setPosition(0);
                wobbleGrabber.setPosition(0);
                sleep(600);
                grabberArm.setPosition(0.5);

                drive.followTrajectory(noneStep4);
                grabberArm.setPosition(0);
                sleep(450);
                wobbleGrabber.setPosition(1);
                sleep(800);
                grabberArm.setPosition(1);

                drive.followTrajectory(noneStep5);

                drive.followTrajectory(noneStep6);
                drive.turn(Math.toRadians(-9.5));

                //fire 1st ring at power shot
                backwardRampDrive();
                sleep(100);
                stopRampDrive();
                launcherMotor.setVelocity(1500);
                sleep(1700);
                forwardRampDrive();
                sleep(600);
                stopRampDrive();
                sleep(200);

                drive.turn(Math.toRadians(7));

                //fire 2nd ring at power shot
                launcherMotor.setVelocity(1600);
                forwardRampDrive();
                sleep(750);
                stopRampDrive();
                sleep(400);

                drive.turn(Math.toRadians(6.5));

                //fire 3rd ring at power shot
                launcherMotor.setVelocity(1600);
                forwardRampDrive();
                sleep(1200);
                stopRampDrive();
                launcherMotor.setPower(0);
                sleep(400);

                drive.followTrajectory(noneStep7);
                grabberArm.setPosition(0.5);
                knockydowny.setPower(-.6);
                sleep(1000);
                knockydowny.setPower(0);
            }
    }
}


