package org.firstinspires.ftc.teamcode.robot.competition.constructor.motors;

import com.qualcomm.robotcore.hardware.DcMotor;

public class mechDriveAuto {
    private DcMotor frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;

    int heading;
    int xVal, yVal, zVal;


    public mechDriveAuto(DcMotor frontLM, DcMotor frontRM, DcMotor rearLM, DcMotor rearRM) {

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
    public void runToPosition (int distance, int direction, double power) {
        int cpr = 538;
        int counts = distance * cpr;

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        switch (direction) {
            case 1:
                // forward
                frontLeftMotor.setTargetPosition(counts);
                frontRightMotor.setTargetPosition(counts);
                rearLeftMotor.setTargetPosition(counts);
                rearRightMotor.setTargetPosition(counts);
                break;
            case 2:
                //back
                frontLeftMotor.setTargetPosition(-counts);
                frontRightMotor.setTargetPosition(-counts);
                rearLeftMotor.setTargetPosition(-counts);
                rearRightMotor.setTargetPosition(-counts);
                break;
            case 3:
                //strafe left
                frontLeftMotor.setTargetPosition(-counts);
                frontRightMotor.setTargetPosition(counts);
                rearLeftMotor.setTargetPosition(counts);
                rearRightMotor.setTargetPosition(-counts);
                break;
            case 4:
                //strafe right
                frontLeftMotor.setTargetPosition(counts);
                frontRightMotor.setTargetPosition(-counts);
                rearLeftMotor.setTargetPosition(-counts);
                rearRightMotor.setTargetPosition(counts);
                break;
            case 5:
                //Rotate left
                frontLeftMotor.setTargetPosition(-counts);
                frontRightMotor.setTargetPosition(counts);
                rearLeftMotor.setTargetPosition(-counts);
                rearRightMotor.setTargetPosition(counts);
                break;
            case 6:
                //Rotate right
                frontLeftMotor.setTargetPosition(counts);
                frontRightMotor.setTargetPosition(-counts);
                rearLeftMotor.setTargetPosition(counts);
                rearRightMotor.setTargetPosition(-counts);
                break;
        }
        while(frontLeftMotor.isBusy() && frontRightMotor.isBusy() && rearLeftMotor.isBusy() && rearRightMotor.isBusy()) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            rearLeftMotor.setPower(power);
            rearRightMotor.setPower(power);

            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            distance = 0;

            frontLeftMotor.setTargetPosition(distance);
            frontRightMotor.setTargetPosition(distance);
            rearLeftMotor.setTargetPosition(distance);
            rearRightMotor.setTargetPosition(distance);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);
        }




    }
    public void powerDrive (int direction, double power) {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        switch (direction) {
            case 1:
                // forward
                frontLeftMotor.setPower(power);
                frontRightMotor.setPower(power);
                rearLeftMotor.setPower(power);
                rearRightMotor.setPower(power);
                break;
            case 2:
                //back
                frontLeftMotor.setPower(-power);
                frontRightMotor.setPower(-power);
                rearLeftMotor.setPower(-power);
                rearRightMotor.setPower(-power);
                break;
            case 3:
                //strafe left
                frontLeftMotor.setPower(-power);
                frontRightMotor.setPower(power);
                rearLeftMotor.setPower(power);
                rearRightMotor.setPower(-power);
                break;
            case 4:
                //strafe right
                frontLeftMotor.setPower(power);
                frontRightMotor.setPower(-power);
                rearLeftMotor.setPower(-power);
                rearRightMotor.setPower(power);
                break;
            case 5:
                //Rotate left
                frontLeftMotor.setPower(-power);
                frontRightMotor.setPower(power);
                rearLeftMotor.setPower(-power);
                rearRightMotor.setPower(power);
                break;
            case 6:
                //Rotate right
                frontLeftMotor.setPower(power);
                frontRightMotor.setPower(-power);
                rearLeftMotor.setPower(power);
                rearRightMotor.setPower(-power);
                break;
        }

    }
    public void stopMotors () {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

}
