package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.vision.pipeline.VisionPipelineYellow;

@TeleOp(name = "AprilTagDrive")
public class AprilTagDriveTrain extends CommandOpMode{

    private Motor fL, bL, fR, bR;
    private VisionPipelineYellow capstoneDetector;
    private GamepadEx gpad;

    @Override
    public void initialize() {

        // Initialize motors
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        // Initialize the vision pipeline with the camera name
        capstoneDetector = new VisionPipelineYellow(hardwareMap, "coolio");
        capstoneDetector.init();

        // Set motor behaviors
        fL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gpad = new GamepadEx(gamepad1);

        // Schedule the telemetry update command
        schedule(new WaitCommand(500).andThen(new RunCommand(() -> {
            telemetry.addData("Is Aligned", capstoneDetector.isAligned());
            telemetry.addData("Rotation Angle", capstoneDetector.getRotationAngle());
            telemetry.addData("Is Centered", capstoneDetector.isCentered());
            telemetry.update();
        })));
    }
}
