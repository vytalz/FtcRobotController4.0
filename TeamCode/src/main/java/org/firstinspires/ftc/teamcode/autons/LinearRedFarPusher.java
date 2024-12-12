package org.firstinspires.ftc.teamcode.autons;

import androidx.core.os.TraceKt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.commands.ShooterCom;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

import java.lang.reflect.WildcardType;

@Autonomous(name = "RedPushLinearPathing")
public class LinearRedFarPusher extends CommandOpMode {
    private MecanumDriveSubsystem drive;
    private Motor frontLeft, frontRight, backLeft, backRight;

    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

    @Override
    public void initialize() {
        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");

        frontLeft.motor.setDirection(DcMotor.Direction.FORWARD);
        frontRight.motor.setDirection(DcMotor.Direction.FORWARD);
        backLeft.motor.setDirection(DcMotor.Direction.FORWARD);
        backRight.motor.setDirection(DcMotor.Direction.FORWARD);

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), true);


//        Trajectory center = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
//                .back(28)
//                .build();
        Trajectory strafeLeft = drive.trajectoryBuilder(startPose, Math.toRadians(90))
                .strafeLeft(60)
                .build();
        Trajectory forward1 = drive.trajectoryBuilder(startPose, Math.toRadians(90))
                .forward(80)
                .build();
        Trajectory strafeRight = drive.trajectoryBuilder(startPose, Math.toRadians(90))
                .strafeRight(60)
                .build();
        Trajectory back = drive.trajectoryBuilder(startPose, Math.toRadians(90))
                .back(80)
                .build();






//        Trajectory setupSample1 = drive.trajectoryBuilder(new Pose2d(37.4, 25.9), Math.toRadians(135))
//                .lineToSplineHeading(new Pose2d(37.4, 45.9, Math.toRadians(135)))
//                .build();

//        Trajectory scoreSample1 = drive.trajectoryBuilder(new Pose2d(42.4, 5.9), Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(42, -63.7), Math.toRadians(270))
//                .build();

//        Trajectory scoreSample1 = drive.trajectoryBuilder(new Pose2d(42.4, 5.9), Math.toRadians(90))
//                .back(60)
//              .build();

//        TrajectoryFollowerCommand autonomous = new TrajectoryFollowerCommand(drive, center);
        TrajectoryFollowerCommand autonomous = new TrajectoryFollowerCommand(drive, strafeLeft);
        TrajectoryFollowerCommand autonomous2 = new TrajectoryFollowerCommand(drive, forward1);
        TrajectoryFollowerCommand autonomous3 = new TrajectoryFollowerCommand(drive,strafeRight);
        TrajectoryFollowerCommand autonomous4 = new TrajectoryFollowerCommand(drive,back);
        //right,back,left, forward

        if(isStopRequested()){

        }

        schedule(autonomous.andThen(new WaitCommand(1000).andThen(autonomous2)).andThen(autonomous3).andThen(autonomous4).andThen(autonomous).andThen(autonomous2));
//        schedule(autonomous2);
//        schedule(autonomous3);
        schedule();
        new WaitCommand(400);


    }
}