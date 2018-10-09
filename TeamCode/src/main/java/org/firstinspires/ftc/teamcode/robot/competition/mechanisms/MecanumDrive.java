package org.firstinspires.ftc.teamcode.robot.competition.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;

//Functions for mecanum drive steering for autonomous and teleop
public class MecanumDrive {

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearRightMotor;
    public DcMotor rearLeftMotor;
    public final DcMotor.RunMode currentMotorRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public static final int TICKS_PER_ROTATION = 375; // TICKS PER ROTATION NEEDED

    //FL is frontleftMotor
    public MecanumDrive(DcMotor FL, DcMotor FR, DcMotor RR, DcMotor RL ) {
        frontLeftMotor = FL;
        frontRightMotor = FR;
        rearRightMotor = RR;
        rearLeftMotor = RL;


        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMotorRunModes(currentMotorRunMode);



    }

    public void setMotorRunModes (DcMotor.RunMode mode) {

        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        rearLeftMotor.setMode(mode);
        rearRightMotor.setMode(mode);

    }

    public void setMotorSpeeds (double speed) {
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        rearRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
    }


    public void driveStraight( double speed, double rotations) {
        int ticks = (int) rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);
        if (rotations < 0 ) { //backwards
            while (frontLeftMotor.getCurrentPosition() > ticks) {
                setMotorSpeeds(-speed);
            }
        } else if (rotations > 0 ){
            while (frontLeftMotor.getCurrentPosition() < ticks) {
                setMotorSpeeds(speed);
            }

        }
    }

    public void strafeLeft (double speed, double rotations) {
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);
        if ( rotations )
    }

    //strafe RIght #TODO

    //rotate #TODO





}
