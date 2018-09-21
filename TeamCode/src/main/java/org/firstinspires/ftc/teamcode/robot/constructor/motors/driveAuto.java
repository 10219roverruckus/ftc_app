package org.firstinspires.ftc.teamcode.robot.constructor.motors;

import com.qualcomm.robotcore.hardware.DcMotor;

public class driveAuto {
    private DcMotor frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;

    int heading;
    int xVal, yVal, zVal;


    public driveAuto (DcMotor frontLM, DcMotor frontRM, DcMotor rearLM, DcMotor rearRM) {

        frontLeftMotor = frontLM;
        frontRightMotor = frontRM;
        rearLeftMotor = rearLM;
        rearRightMotor = rearRM;

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
