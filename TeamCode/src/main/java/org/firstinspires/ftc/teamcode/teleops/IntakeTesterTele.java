package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemFC;
import org.firstinspires.ftc.teamcode.vision.pipeline.VisionPipelineRed;
//import org.firstinspires.ftc.teamcode.vision.pipeline.VisionPipelineRed;

@TeleOp(name = "IntakeTester")
public class IntakeTesterTele extends CommandOpMode{

//    private Motor fL, bL, fR, bR;

//    private RevIMU revIMU;
//    private Motor intakeActuator;

//    private Motor outtakeActuator;


//    private DriveCommand driveC;

//    private DriveSubsystemFC driveS;
//    private VisionPipelineRed capstoneDetector;

    private CRServo clawArm;

    private CRServo wristAxial;
    private CRServo wristRotational;

    private SimpleServo clawFinger;
    private GamepadEx gunnerPad;

    private GamepadEx driverPad;

    private Double axialControl;

    private Double rotationalControl;

    private Double armControl;

    @Override
    public void initialize() {

        // Initialize motors
//        fL = new Motor(hardwareMap, "fL");
//        fR = new Motor(hardwareMap, "fR");
//        bL = new Motor(hardwareMap, "bL");
//        bR = new Motor(hardwareMap, "bR");

//
//        intakeActuator = new Motor(hardwareMap, "intakeSlides");
//        outtakeActuator = new Motor(hardwareMap, "outtakeSlides");

        clawArm = hardwareMap.get(CRServo.class, "armit");
        wristAxial = hardwareMap.get(CRServo.class, "axial");
        wristRotational = hardwareMap.get(CRServo.class, "turret");
        clawFinger = new SimpleServo(hardwareMap,"clawit", -1, 1);



        //  Initialize the vision pipeline with the camera name
//        capstoneDetector = new VisionPipelineRed(hardwareMap, "coolio");
//        capstoneDetector.init();

//        revIMU = new RevIMU(hardwareMap);
//        revIMU.init();
//
//        fL.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fR.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bL.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bR.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Set motor behaviors
//        fL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        driverPad = new GamepadEx(gamepad1);
        gunnerPad = new GamepadEx(gamepad2);

        schedule(new RunCommand(() -> {
            rotationalControl = gunnerPad.getRightX();
            axialControl = gunnerPad.getRightY();
            armControl = gunnerPad.getLeftY();

            wristAxial.setPower(-axialControl);
            wristRotational.setPower(rotationalControl);
            clawArm.setPower(-armControl);

        }));


//        double rotationalControl = gunnerPad.getRightX();
//        double axialControl = gunnerPad.getRightY();


//        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new InstantCommand(() -> {
//                    if (clawWrist.getPosition()==1) {
//                        // If claw is open, close it
//                        clawWrist.setPosition(0);
//                    } else {
//                        // If claw is closed, open it
//                        clawWrist.setPosition(1);
//                    }
//                    // Toggle the state
//                    ;
//                })
//        );

        gunnerPad.getGamepadButton(GamepadKeys.Button.B).whenPressed
                (
                        new InstantCommand(() -> clawFinger.rotateByAngle(0.5)));

        gunnerPad.getGamepadButton(GamepadKeys.Button.Y).whenPressed
                (
                        new InstantCommand(() -> clawFinger.rotateByAngle(-0.5)));

        gunnerPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld               (
//                        new InstantCommand(() -> armRotation.rotateByAngle(15)).andThen(new InstantCommand(() -> armRotation2.rotateByAngle(-15))));
                      new StartEndCommand(() -> wristAxial.setPower(1), () -> wristAxial.setPower(0)));


        gunnerPad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld
                (  new StartEndCommand(() -> wristAxial.setPower(-1), () -> wristAxial.setPower(0)));




//                        new InstantCommand(() -> armRotation.rotateByAngle(-15)).andThen(new InstantCommand(() -> armRotation2.rotateByAngle(15))));

//        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new DriveCommand(driveS, driverPad::getLeftX, driverPad::getLeftY, driverPad::getRightX,0.5));


//        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed
//                (
//                    new InstantCommand(()-> { if (clawWrist.getAngle() == 0)
//                        clawWrist.turnToAngle(45);
//                    } else { clawWrist.turnToAngle(0);
//
//        }
//
//                );

//        driveS = new DriveSubsystemFC(fL, fR, bL, bR, revIMU, 0.9);
//        driveC = new DriveCommand(driveS, driverPad::getLeftY, driverPad::getLeftX, driverPad::getRightX, 0.9);
//
//        register(driveS);
//        driveS.setDefaultCommand(driveC);





//        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new StartEndCommand(() -> outtakeActuator.set(1), () -> outtakeActuator.stopMotor()));

//        FtcDashboard.getInstance().startCameraStream(capstoneDetector.getCamera(),30);

//         Schedule the telemetry update command
        schedule(new WaitCommand(500).andThen(new RunCommand(() -> {



//            telemetry.addData("Is Aligned", capstoneDetector.isAligned());
//            telemetry.addData("Rotation Angle", capstoneDetector.getRotationAngle());
//            telemetry.addData("Is Centered", capstoneDetector.isCentered());
//            telemetry.addData("Arm servo 1 angle", armRotation.getAngle());
//            telemetry.addData("Arm servo 2 angle", armRotation2                                                                                                                                                         .getAngle());
//
            telemetry.addData("Servo angle:", clawFinger.getAngle());
            telemetry.addData("Right Joystick Y", axialControl);
            telemetry.addData("Right Joystick X", rotationalControl);
            telemetry.update();
        })));
    }
}