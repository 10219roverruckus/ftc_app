package org.firstinspires.ftc.teamcode.robot.competition.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//Functions for mecanum drive steering for autonomous and teleop
public class MecanumDrive {

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearRightMotor;
    public DcMotor rearLeftMotor;
    public final DcMotor.RunMode currentMotorRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public static final int TICKS_PER_ROTATION = 375; // TICKS PER ROTATION NEEDED!!!!!!!!!!!!!! :)


    public int counts = counts * cpr; // added counts in here
    public int cpr = 538;

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
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMotorRunModes(currentMotorRunMode);



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

    //Driving Forward
    public void driveForward( double speed, double rotations) {
        int ticks = (int) rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

        if (rotations < 0 && linearOp.opModeIsActive()) { //backwards
            while (frontLeftMotor.getCurrentPosition() > ticks) {
                setMotorSpeeds(speed);
            }
        }
    }

     // Driving Backward
    public void driveBackward ( double speed, double rotations){

        int ticks = (int) rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

        if (rotations > 0 && linearOp.opModeIsActive()) {
            while (frontLeftMotor.getCurrentPosition() < ticks) {
                setMotorSpeeds(-speed);
            }
        }
    }

    // Strafing left
    public void strafeLeft (double speed, double rotations) {
        int ticks = (int) rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

        if (rotations < 0 && linearOp.opModeIsActive()) {
            while (frontLeftMotor.getCurrentPosition() > ticks) {
                frontLeftMotor.setTargetPosition(-counts);
                frontRightMotor.setTargetPosition(counts);
                rearLeftMotor.setTargetPosition(counts);
                rearRightMotor.setTargetPosition(-counts);
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
                frontLeftMotor.setTargetPosition(counts);
                frontRightMotor.setTargetPosition(-counts);
                rearLeftMotor.setTargetPosition(-counts);
                rearRightMotor.setTargetPosition(counts);
            }
        }
    }

    // Rotating counterclockwise
    public void rotateLeft (double speed, double rotations) {
        int ticks = (int) rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);

        if (rotations < 0 && linearOp.opModeIsActive()) {   // HELP
            while (frontLeftMotor.getCurrentPosition() > ticks) { // HELP
                frontLeftMotor.setTargetPosition(-counts);
                frontRightMotor.setTargetPosition(counts);
                rearLeftMotor.setTargetPosition(-counts);
                rearRightMotor.setTargetPosition(counts);
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
                frontLeftMotor.setTargetPosition(counts);
                frontRightMotor.setTargetPosition(-counts);
                rearLeftMotor.setTargetPosition(counts);
                rearRightMotor.setTargetPosition(-counts);
            }
        }
    }







}
