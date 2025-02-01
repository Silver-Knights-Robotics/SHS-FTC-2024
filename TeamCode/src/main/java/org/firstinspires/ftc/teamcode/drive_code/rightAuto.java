package org.firstinspires.ftc.teamcode.drive_code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE-by-DISTANCE square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@Config
@Autonomous(group = "test_auto")
public class rightAuto extends LinearOpMode {
    public static double DISTANCE = 96; // in
    private DcMotor rotate1 = null;
    private DcMotor rotate2 = null;
    private DcMotor armUp = null;
    private Servo wrist = null;
    private Servo claw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(-24, 65, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        rotate1 = hardwareMap.get(DcMotor.class, "rotate1");
        rotate2 = hardwareMap.get(DcMotor.class, "rotate2");
        armUp = hardwareMap.get(DcMotor.class, "armUp");

        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        rotate1.setDirection(DcMotor.Direction.FORWARD);
        rotate2.setDirection(DcMotor.Direction.REVERSE);
        armUp.setDirection(DcMotor.Direction.FORWARD);

        rotate1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotate1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        claw.setPosition(0.9289);
        wrist.setPosition(0.3039);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-6.5, 33.5, Math.toRadians(90)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(31)
                .build();

        Trajectory block1setup = drive.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(-36, 16))
                .splineToConstantHeading(new Vector2d(-48, 16), Math.toRadians(90))
                .build();

//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .back(24)
//                .build();
//
//        //.plus(new Pose2d(0, 0, Math.toRadians(180))), false
//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .strafeLeft(10)
//                .build();

        Trajectory traj5 = drive.trajectoryBuilder(block1setup.end())
                .forward(46)
                .build();

//        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                .back(46)
//                .build();
//
//        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
//                .strafeLeft(9)
//                .build();

        Trajectory block2setup = drive.trajectoryBuilder(traj5.end())
                .lineToConstantHeading(new Vector2d(-50, 14))
                .splineToConstantHeading(new Vector2d(-58, 14), Math.toRadians(90))
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(block2setup.end())
                .forward(46)
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .back(46)
                .build();

        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .strafeLeft(8)
                .build();

        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .forward(46)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
//        sleep(1000);
        scoreSpecimen();
        wrist.setPosition(0.3039);
        drive.followTrajectory(traj2);
//        drive.followTrajectory(traj3);
////        drive.turn(Math.toRadians(180 - 1e-10));
//        drive.followTrajectory(traj4);
        drive.followTrajectory(block1setup);
        drive.followTrajectory(traj5);
//        drive.followTrajectory(traj6);
//        drive.followTrajectory(traj7);
        drive.followTrajectory(block2setup);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        drive.followTrajectory(traj10);
        drive.followTrajectory(traj11);
    }

    public void scoreSpecimen(){

        resetRuntime();

        wrist.setPosition(0.4);

        rotate1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotate1.setTargetPosition(-465);
        rotate2.setTargetPosition(-440);
        armUp.setTargetPosition(165);

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

        while (opModeIsActive() && getRuntime()<3.7){
            telemetry.addLine("waiting");
            telemetry.update();
        }

        claw.setPosition(0.5006);

        while(opModeIsActive() && claw.getPosition() > 0.5006){
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

//        rotate1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rotate2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rotate1.setPower(0.2);
//        rotate2.setPower(0.2);
//
//        while (true) {
//            telemetry.addLine("Rotating arm");
//            telemetry.update();
//        }
//
//        while (opModeIsActive() && armUp.isBusy()) {
//            telemetry.addLine("Moving arm");
//            telemetry.update();
//        }
//
//        wrist.setPosition(0.8156);
//
//        armUp.setTargetPosition(0);
//
//        armUp.setPower(-0.1);
//
//        claw.setPosition(0.5006);
//
//        while (claw.getPosition() > 0.55){
//            telemetry.addLine("Opening claw");
//            telemetry.update();
//        }
////        resetRuntime();
////        if (getRuntime() > 0.5){
////            claw.setPosition(0.5006);
////        }
//
//        rotate1.setTargetPosition(0);
//        rotate2.setTargetPosition(0);
//
//        rotate1.setPower(-0.5);
//        rotate2.setPower(-0.5);
//
//        while (opModeIsActive() && rotate1.isBusy() && rotate2.isBusy()) {
//            telemetry.addLine("Rotating arm");
//            telemetry.update();
//        }
//
////        rotate1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        rotate2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        armUp.setMode();
//
//        rotate1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rotate2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        wrist.setPosition(1);
    }
}
