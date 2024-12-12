package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveSubsystemFC extends SubsystemBase {

    private Motor fL, fR, bL, bR;

    private RevIMU imu;

    private double mult;

    private MecanumDrive mDrive;


    public DriveSubsystemFC(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, RevIMU revIMU, double multiplier){
        fL = frontLeft;
        fR = frontRight;
        bL = backLeft;
        bR = backRight;

        mult = multiplier;

        imu = revIMU;

        mDrive = new MecanumDrive(fL, fR, bL, bR);
    }


    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed){
        mDrive.driveFieldCentric(strafeSpeed*mult, forwardSpeed*mult, turnSpeed*mult, imu.getRotation2d().getDegrees(), false);

    }





}
