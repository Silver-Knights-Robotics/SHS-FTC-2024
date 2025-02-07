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

package org.firstinspires.ftc.teamcode.drive_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="blue_teleop", group="Linear OpMode")
//@Disabled
public class blue_teleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rotate1 = null;
    private DcMotor rotate2 = null;
    private DcMotor armUp = null;
    private Servo wrist = null;
    private Servo claw = null;

    private Boolean open = false;

    private int button = 1;
    private Boolean newPose = false;

    Pose2d specimenPose = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "white");
        leftBackDrive = hardwareMap.get(DcMotor.class, "yellow");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "blue");
        rightBackDrive = hardwareMap.get(DcMotor.class, "black");

        rotate1 = hardwareMap.get(DcMotor.class, "rotate1");
        rotate2 = hardwareMap.get(DcMotor.class, "rotate2");
        armUp = hardwareMap.get(DcMotor.class, "armUp");

        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        rotate1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotate1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotate2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotate1.setDirection(DcMotor.Direction.FORWARD);
        rotate2.setDirection(DcMotor.Direction.REVERSE);
        armUp.setDirection(DcMotor.Direction.FORWARD);

        rotate1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ROAD RUNNER THINGS HERE
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        drive.setPoseEstimate(PoseStorage.currentPose);

//        startingWristPosition();
//        wrist.setPosition(0.5);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Pose2d specimenPose = new Pose2d(-47, 54, Math.toRadians(90));

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//            Pose2d specimenPose;

            //ROAD RUNNER
            if(gamepad1.a){
                drive.setPoseEstimate(new Pose2d(-47.01, 54.01, Math.toRadians(90)));
            }

            telemetry.addData("Specimen Pose", specimenPose);

            Pose2d myPose = drive.getPoseEstimate();
            drive.update();

            Trajectory grabSpecimen = drive.trajectoryBuilder(drive.getPoseEstimate())
//                    .splineTo(new Vector2d(-46.5, 55), Math.toRadians(90))
                    .lineToLinearHeading(specimenPose)
                    //open the claw
                    .addDisplacementMarker(() -> {
                        claw.setPosition(0.5006);
                    })
                    .build();

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // gamepad2 left joystick y rotate
            // gamepad2 right joystick y up

            double rotate = gamepad2.left_stick_y;
            double up = -gamepad2.right_stick_y;

            rotate1.setPower(rotate * 0.4);
            rotate2.setPower(rotate * 0.4);
            armUp.setPower(up * 0.6);

            // Wrist: left trigger down, left bumper up

            if (gamepad2.y && wrist.getPosition() >= 0.3039) {
                wrist.setPosition(wrist.getPosition() - 0.012);
            } else {
                wrist.setPosition(wrist.getPosition());
            }
            if (gamepad2.a && wrist.getPosition() <= 0.7711) {
                wrist.setPosition(wrist.getPosition() + 0.012);
            } else {
                wrist.setPosition(wrist.getPosition());
            }

            // Claw: right bumper opens, right trigger closes

            if (gamepad2.b) {
                claw.setPosition(0.9294);
            }
            if (gamepad2.x) {
                claw.setPosition(0.5006);
            }

            if (gamepad2.right_trigger != 0){
                scoreSpecimen();
            }
//
////                button = 0;
//                if (open && claw.getPosition() == 0.5006){
//                    claw.setPosition(0.9294);
//                    open = false;
//                }
//                else if(!open && claw.getPosition() == 0.9294){
//                    claw.setPosition(0.5006);
//                    open = true;
//                }
//                button = 1;
//            }


            if (gamepad2.right_bumper) {
                wrist.setPosition(0.6839);
            }

            //Road Runner Stuff

            if (gamepad1.right_trigger != 0) {
                drive.followTrajectory(grabSpecimen);
            }

            if (gamepad1.dpad_down) {
                drive.breakFollowing();
            }

            telemetry.addData("Wrist Location", wrist.getPosition());
            telemetry.addData("Claw Location", claw.getPosition());

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * 0.8);
            rightFrontDrive.setPower(rightFrontPower * 0.8);
            leftBackDrive.setPower(leftBackPower * 0.8);
            rightBackDrive.setPower(rightBackPower * 0.8);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

            //Road Runner Position
            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());

            telemetry.addData("rotate1 encoder position", rotate1.getCurrentPosition());
            telemetry.addData("rotate2 encoder position", rotate2.getCurrentPosition());
            telemetry.addData("armUp encoder position", armUp.getCurrentPosition());
        }
    }
    public void startingWristPosition(){
        wrist.setPosition(1);
    }
    public void scoreSpecimen() {

        resetRuntime();

        wrist.setPosition(0.4);

        rotate1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotate1.setTargetPosition(-465);
        rotate2.setTargetPosition(-440);
        armUp.setTargetPosition(155);

        rotate1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotate2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Add power to the arms in order for them to move!!!!

        rotate1.setPower(0.2);
        rotate2.setPower(0.2);
        armUp.setPower(0.2);

        while (opModeIsActive() && rotate1.isBusy() && rotate2.isBusy()) {
            telemetry.addLine("Rotating arm");
            telemetry.update();
        }

        armUp.setPower(0);

        armUp.setTargetPosition(0);

        armUp.setPower(0.2);

        while (opModeIsActive() && armUp.isBusy()) {
            telemetry.addLine("Rotating arm");
            telemetry.update();
        }

        telemetry.addData("runtime", getRuntime());
        telemetry.update();

        while (opModeIsActive() && getRuntime() < 3.65) {
            telemetry.addLine("waiting");
            telemetry.update();
        }

        claw.setPosition(0.5006);

        while (opModeIsActive() && claw.getPosition() > 0.5006) {
            telemetry.addLine("waiting");
            telemetry.update();
        }

        rotate1.setPower(0);
        rotate2.setPower(0);

        rotate1.setTargetPosition(0);
        rotate2.setTargetPosition(0);

        rotate1.setPower(0.3);
        rotate2.setPower(0.3);

        while (opModeIsActive() && rotate1.isBusy() && rotate2.isBusy()) {
            telemetry.addLine("Rotating arm");
            telemetry.update();
        }

        rotate1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotate2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotate1.setDirection(DcMotor.Direction.FORWARD);
        rotate2.setDirection(DcMotor.Direction.REVERSE);
        armUp.setDirection(DcMotor.Direction.FORWARD);

        rotate1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
