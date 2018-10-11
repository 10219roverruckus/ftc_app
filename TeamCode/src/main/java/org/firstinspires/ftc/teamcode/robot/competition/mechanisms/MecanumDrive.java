package org.firstinspires.ftc.teamcode.robot.competition.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Functions for mecanum drive steering for autonomous and teleop
public class MecanumDrive {

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearRightMotor;
    public DcMotor rearLeftMotor;
    public final DcMotor.RunMode currentMotorRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public static final int TICKS_PER_ROTATION = 375; // TICKS PER ROTATION NEEDED!!!!!!!! :)

    public int cpr = 538;
    public int counts;

    public LinearOpMode linearOp = null;



// To stop loop when stop button pressed.
    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }



    public MecanumDrive(DcMotor FL, DcMotor FR, DcMotor RR, DcMotor RL ) {
        frontLeftMotor = FL;      //FL is front left Motor
        frontRightMotor = FR;     // FR is front Right Motor
        rearRightMotor = RR;       //RR is back or rear right motor
        rearLeftMotor = RL;         // RL is back or rear left motor


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMotorRunModes(currentMotorRunMode);
        counts = counts * cpr; // added counts in here or should it be ticks



    }
    public void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

    }
// how do I describe a mode
    public void setMotorRunModes (DcMotor.RunMode mode) {

        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        rearLeftMotor.setMode(mode);
        rearRightMotor.setMode(mode);

    }

    // default speed for all motors
    public void setMotorSpeeds (double speed) {
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        rearRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
    }

    public void setMotorSpeedStrafeLeft(double speed) {
    frontLeftMotor.setPower(-speed);
    frontRightMotor.setPower(speed);
    rearLeftMotor.setPower(speed);
    rearRightMotor.setPower(-speed);
    }

    public void setMotorSpeedStrafeRight(double speed) {
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        rearLeftMotor.setPower(-speed);
        rearRightMotor.setPower(speed);
    }

    public void setMotorSpeedRotateLeft(double speed) {
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        rearLeftMotor.setPower(-speed);
        rearRightMotor.setPower(speed);
    }

    public void setMotorSpeedRotateRight(double speed) {
        frontLeftMotor.setTargetPosition(counts);
        frontRightMotor.setTargetPosition(-counts);
        rearLeftMotor.setTargetPosition(counts);
        rearRightMotor.setTargetPosition(-counts);
    }

    public void FORWARDPOWER () {
        frontRightMotor.setPower(.5);
        frontLeftMotor.setPower(.5);
        rearLeftMotor.setPower(.5);
        rearRightMotor.setPower(.5);

    }

    //Driving Forward
    public void driveForward( double speed, double rotations) {
        int ticks = (int) rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);


            while (frontLeftMotor.getCurrentPosition() < ticks && linearOp.opModeIsActive()) {
                setMotorSpeeds(speed);
            }
        }


     // Driving Backward
    public void driveBackward ( double speed, double rotations){

        int ticks = (int) rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

            while (frontLeftMotor.getCurrentPosition() < ticks && linearOp.opModeIsActive() ) {
                setMotorSpeeds(-speed);
            }
            stopMotors();
        }


    // Strafing left
    public void strafeLeft (double speed, double rotations) {
        int ticks = (int) rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

        if (rotations < 0 && linearOp.opModeIsActive()) {
            while (frontLeftMotor.getCurrentPosition() > ticks) {
                setMotorSpeedStrafeLeft(speed);
            }
        }
    }



    // Strafing Right
     public void strafeRight (double speed, double rotations) {
         int ticks = (int) rotations * TICKS_PER_ROTATION;
         setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

        if (rotations > 0 && linearOp.opModeIsActive()) {
            while(frontLeftMotor.getCurrentPosition() < ticks) {
                setMotorSpeedStrafeLeft(speed);
            }
        }
    }

    // Rotating counterclockwise
    public void RotateLeft (double speed, double rotations) {
        int ticks = (int) rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

        if (rotations > 0 && linearOp.opModeIsActive()) {   // HELP
            while (frontLeftMotor.getCurrentPosition() > ticks) { // HELP
            setMotorSpeedRotateLeft(speed);
            }
        }
    }
    // rotating clockwise
    public void rotateRight (double speed, double rotations) {
        int ticks = (int) rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

        if(rotations > 0 && linearOp.opModeIsActive()){ //HELP
            while (frontLeftMotor.getCurrentPosition() < ticks) { //HELP
            setMotorSpeedRotateRight(speed);
            }
        }
    }



//    public void TeamMarkerToCrater () {
//
//        rotateLeft(.5, 2);
//
//        while (distanceSensor > 5 ) {  // distance or color sensor needs to be determined
//            driveForward(.5, 4);
//        }
//        driveForward(.5,1);
//
//    }








}
