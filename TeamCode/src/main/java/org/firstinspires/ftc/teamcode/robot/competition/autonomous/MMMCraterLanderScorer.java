package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotaterServos;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeSpinnerMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.MineralLift;

public class MMMCraterLanderScorer {
    public MMMCraterLanderScorer () {

    }

    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;

    public final int RED_THRESHOLD = 30;                //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100;              //maybe a higher threshold (12.5)
    float hsvValues[] = {0F, 0F, 0F};

    // created constant variables that are used for speed (different setting)

    final double SPD_DRIVE_LOW = .38;                  //Lowest speed
    final double SPD_DRIVE_MED = 0.6;   //was .4;                   Default is  SPD_MED
    final double SPD_DRIVE_HIGH = .80;
    final double SPD_DRIVE_MAX = 1.0;
    final double SPD_ARM_MED = .5;
    final long sleepTime = 50;
    final int servoRotateTeamMarker = 1000;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }


    public void findingMineralCamera(double cameraGoldLocation) {

        //find location of the mineral using camera

        if (cameraGoldLocation < 200 && cameraGoldLocation > 1) {
            goldPosition = GoldPosition.RIGHT;                           //commented out while camera does not work
        } else if (cameraGoldLocation > 200) {                            //program works perfectly DO NOT CHANGE THE CODE
            goldPosition = GoldPosition.MIDDLE;
        } else {
            goldPosition = GoldPosition.LEFT;
        }
//        goldPosition = GoldPosition.LEFT;
    }

    public void hookDriveOff (GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotaterServos myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeSpinnerMotor myIntakeSpinnerMotor) {

        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
        linearOp.sleep(sleepTime);
        //Change values if done so in MecanumMineralMinerCrater!
        myMechDrive.driveForward (SPD_DRIVE_MED, .5);                  // get away from the lander
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .6);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);
        myMechDrive.driveBackward(SPD_DRIVE_HIGH, .51);
        linearOp.sleep(sleepTime);
    }


    public void driveAwayFromHook (GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotaterServos myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeSpinnerMotor myIntakeSpinnerMotor) {


    }

    public void codeExamples (GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotaterServos myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeSpinnerMotor myIntakeSpinnerMotor, MineralLift myMineralLift) {

        //GYRO TURNS
        //Use the following 3 lines of code for any Gyro Turns:
        myGyro.gyroOrientMecanum(-90, myMechDrive);    //Angle the robot turns.
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        //EXTENDER
        //use encoders to intake
        myIntakeExtenderArm.extendIntakeArm(1000, SPD_ARM_MED, 2);
        linearOp.sleep(sleepTime);
        //use encoders to retract
        myIntakeExtenderArm.retractIntakeArm(0,SPD_ARM_MED,2);
        linearOp.sleep(sleepTime);
        //use power & time to retract
        myIntakeExtenderArm.retractPowerAuto(1);
        linearOp.sleep(500);
        myIntakeExtenderArm.stopIntakeArm();
        //use power & time to EXTEND - use negative power.
        myIntakeExtenderArm.retractPowerAuto(-1);
        linearOp.sleep(500);
        myIntakeExtenderArm.stopIntakeArm();

        //MINERAL LIFT
            //raise mineral lift for 'x' time
        myMineralLift.RaiseMineralLift();
        linearOp.sleep(1000);
        myMineralLift.stopMotors();
            //retract mineral lift for 'y' time
        myMineralLift.LowerMineralLift();
        linearOp.sleep(600);
        myMineralLift.stopMotors();

        //RAISE & LOWER INTAKE
            //LOWER INTAKE
        myIntakeRotator.loweredRotater();                // lower rotater - uses Servos
        linearOp.sleep(sleepTime);
            //RAISE INTAKE
        myIntakeRotator.raisedRotater();                                        //raise rotater
        linearOp.sleep(sleepTime);

        //SPINNER MOTOR THING
            //INTAKE!!! for sleep
            //positive power for forward
        myIntakeSpinnerMotor.intakeSpinner(1);                         //start spinner
        linearOp.sleep(1000);
        myIntakeSpinnerMotor.stopMotors();
            //REVERSE!!! for sleep
            //negative power for reverse
        myIntakeSpinnerMotor.intakeSpinner(-1);
        linearOp.sleep(1000);
        myIntakeSpinnerMotor.stopMotors();
    }
}
