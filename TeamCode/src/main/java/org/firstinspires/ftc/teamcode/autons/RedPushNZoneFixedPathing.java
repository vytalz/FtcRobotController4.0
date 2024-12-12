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

@Autonomous(name = "RedNZoneFixedPathing")
public class RedPushNZoneFixedPathing extends CommandOpMode {
    private MecanumDriveSubsystem drive;
    private Motor frontLeft, frontRight, backLeft, backRight;

    private Pose2d startPose = new Pose2d(-12.42, -61.11, Math.toRadians(90));

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

        Trajectory splineToSampleOne = drive.trajectoryBuilder(startPose, Math.toRadians(90))
                .splineTo(new Vector2d(-42.72, -15.47),Math.toRadians(-90))
                .build();

        Trajectory scoreSampleOne = drive.trajectoryBuilder(new Pose2d(-42.72, -15.47), Math.toRadians(-90))
                .splineTo(new Vector2d(-59.2, -54.1), Math.toRadians(-90))
                .build();

        Trajectory resetSampleTwo = drive.trajectoryBuilder(new Pose2d(-59.2, -54.1), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-30.32, -17.37), Math.toRadians(-90))
                .build();

        Trajectory setUpSampleTwo = drive.trajectoryBuilder(new Pose2d(-30.32, -17.37), Math.toRadians(-90))
                .splineTo(new Vector2d(-52.9, -10.3), Math.toRadians(-90))
                .build();

        Trajectory scoreSampleTwo = drive.trajectoryBuilder(new Pose2d(-52.9, -10.3), Math.toRadians(-90))
                .splineTo(new Vector2d(-57.01, -58.47), Math.toRadians(-90))
                .build();




        TrajectoryFollowerCommand autonomous = new TrajectoryFollowerCommand(drive, splineToSampleOne);
        TrajectoryFollowerCommand autonomous1 = new TrajectoryFollowerCommand(drive, scoreSampleOne);
        TrajectoryFollowerCommand autonomous2 = new TrajectoryFollowerCommand(drive, resetSampleTwo);
        TrajectoryFollowerCommand autonomous3 = new TrajectoryFollowerCommand(drive, setUpSampleTwo);
        TrajectoryFollowerCommand autonomous4 = new TrajectoryFollowerCommand(drive, scoreSampleTwo);

        if(isStopRequested()){

        }

        schedule(autonomous.andThen(autonomous1.andThen(autonomous2.andThen(autonomous3.andThen(autonomous4)))));
//        schedule(autonomous2);
//        schedule(autonomous3);
        schedule();
        new WaitCommand(400);


    }
}