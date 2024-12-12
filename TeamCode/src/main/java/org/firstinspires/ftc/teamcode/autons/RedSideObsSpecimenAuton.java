package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@Autonomous(name = "RedObsSpecimenAuton")
public class RedSideObsSpecimenAuton extends CommandOpMode {
    private MecanumDriveSubsystem drive;
    private Motor frontLeft, frontRight, backLeft, backRight;

    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

    private SimpleServo clawWrist;

    private SimpleServo armRotation;
    private SimpleServo armRotation2;

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

        armRotation = new SimpleServo(hardwareMap, "arm", 0, 360);
        armRotation2 = new SimpleServo(hardwareMap, "arm2", 0, 360);
        clawWrist = new SimpleServo(hardwareMap, "claw", -1, 1);


        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), true);


//        Trajectory center = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
//                .back(28)
//                .build();
        Trajectory centerSetUp = drive.trajectoryBuilder(startPose, Math.toRadians(90))
                .strafeLeft(25)
                .build();

        Trajectory forward = drive.trajectoryBuilder(startPose, Math.toRadians(90))
                .forward(60)
                .build();
        Trajectory back = drive.trajectoryBuilder(startPose, Math.toRadians(90))
                .back(55)
                .build();

        Trajectory strafeToPark = drive.trajectoryBuilder(startPose, Math.toRadians(90))
                .strafeRight(120)
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

        TrajectoryFollowerCommand centering = new TrajectoryFollowerCommand(drive, centerSetUp);
        TrajectoryFollowerCommand comeToScore = new TrajectoryFollowerCommand(drive, forward);
        TrajectoryFollowerCommand backUp = new TrajectoryFollowerCommand(drive, back);
        TrajectoryFollowerCommand park = new TrajectoryFollowerCommand(drive, strafeToPark);

        if(isStopRequested()){

        }

        schedule(
                new SequentialCommandGroup(
                        centering,
                        new InstantCommand(() -> armRotation.rotateByAngle(-100)).andThen(new InstantCommand(() -> armRotation2.rotateByAngle(100))),
                        comeToScore,
                        new InstantCommand(() -> armRotation.rotateByAngle(280)).andThen(new InstantCommand(() -> armRotation2.rotateByAngle(-280))),
                        new WaitCommand(500),
                        new InstantCommand(() -> clawWrist.rotateByAngle(-0.5)),
                        new WaitCommand(500),
                        new InstantCommand(() -> armRotation.rotateByAngle(-45)).andThen(new InstantCommand(() -> armRotation2.rotateByAngle(45))),
                        backUp,
                        park



                ));




//        schedule(autonomous2);
//        schedule(autonomous3);

    }
}