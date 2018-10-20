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
    public static final double TICKS_PER_ROTATION = 538; // TICKS (COUNTS) PER ROTATION NEEDED!!!!!!!! :)
    // http://www.andymark.com/NeveRest-20-12V-Planetary-Gearmotor-p/am-3637.htm

    //public int cpr = 538;
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


        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMotorRunModes(currentMotorRunMode);
        //counts = counts * cpr; // added counts in here or should it be ticks



    }
    public void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

    }

    public void alignLeftFront (double speed) {
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
    }

    public void alignLeftBack (double speed) {
        rearLeftMotor.setPower(-speed);
        rearRightMotor.setPower(speed);
    }
    public void alignRightFront (double speed) {
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
    }
    public void alignRightBack (double speed) {
        rearLeftMotor.setPower(speed);
        rearRightMotor.setPower(-speed);
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

    public void setMotorPowerStrafeRight (double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(-power);
    }

    public void setMotorPowerStrafeLeft (double power) {
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(-power);
        rearRightMotor.setPower(power);
    }

    //Driving Forward
    public void driveForward( double speed, double rotations) {

        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);


            while (frontLeftMotor.getCurrentPosition() < ticks && linearOp.opModeIsActive()) {
                setMotorSpeeds(speed);
            }
            stopMotors();
        }


     // Driving Backward
    public void driveBackward ( double speed, double rotations){

        double ticks = rotations * (-1) * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

            while (frontLeftMotor.getCurrentPosition() > ticks && linearOp.opModeIsActive()) {
                setMotorSpeeds(-speed);
            }
            stopMotors();
        }


    // Strafing left
    public void rotateLeft (double speed, double rotations) {
        double ticks = Math.abs(rotations) * (-1) * TICKS_PER_ROTATION; //strafing left moves encoder towards positive infinity
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

            while (frontLeftMotor.getCurrentPosition() > ticks && linearOp.opModeIsActive()) {
                frontLeftMotor.setPower(-speed);
                frontRightMotor.setPower(speed);
                rearLeftMotor.setPower(speed);
                rearRightMotor.setPower(-speed);
            }
            stopMotors();
    }



    // Strafing Right
     public void rotateRight (double speed, double rotations) {
         double ticks = Math.abs(rotations) * TICKS_PER_ROTATION; //strafing right moves encoder towards -infinity
         setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         setMotorRunModes(currentMotorRunMode);

            while(frontLeftMotor.getCurrentPosition() < ticks && linearOp.opModeIsActive()) {
                linearOp.telemetry.addData("current position", frontLeftMotor.getCurrentPosition());
                linearOp.telemetry.update();

                frontLeftMotor.setPower(speed);
                frontRightMotor.setPower(-speed);
                rearLeftMotor.setPower(-speed);
                rearRightMotor.setPower(speed);
            }
            stopMotors();
    }

    // Rotating counterclockwise
    public void strafeRight (double speed, double rotations) {
        double ticks = Math.abs(rotations) * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

            while (frontLeftMotor.getCurrentPosition() < ticks && linearOp.opModeIsActive()) {
                frontLeftMotor.setPower(speed);
                frontRightMotor.setPower(-speed);
                rearLeftMotor.setPower(speed);
                rearRightMotor.setPower(-speed);
            }
            stopMotors();
    }
    // rotating clockwise
    public void strafeLeft (double speed, double rotations) {
        double ticks = Math.abs(rotations) * (-1) *  TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

            while (frontLeftMotor.getCurrentPosition() > ticks && linearOp.opModeIsActive()) {
                frontLeftMotor.setPower(-speed);
                frontRightMotor.setPower(speed);
                rearLeftMotor.setPower(-speed);
                rearRightMotor.setPower(speed);
            }
        stopMotors();
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
