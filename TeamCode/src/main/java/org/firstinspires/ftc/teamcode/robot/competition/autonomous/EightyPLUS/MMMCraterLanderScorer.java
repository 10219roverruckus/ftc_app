package org.firstinspires.ftc.teamcode.robot.competition.autonomous.EightyPLUS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.autonomous.GoldPosition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotaterServos;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeSpinnerMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LanderServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.MineralLift;
import org.firstinspires.ftc.teamcode.robot.old.IntakeRotator;


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
    final long sleepTime = 20;
    final int servoRotateTeamMarker = 1000;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }


    public void findingMineralCamera(double cameraGoldLocation) {

        //find location of the mineral using camera

        if (cameraGoldLocation < 320 && cameraGoldLocation > 50) {
            goldPosition = GoldPosition.MIDDLE;                           //commented out while camera does not work
        } else if (cameraGoldLocation > 320) {                            //program works perfectly DO NOT CHANGE THE CODE
            goldPosition = GoldPosition.RIGHT;
        } else {
            goldPosition = GoldPosition.LEFT;
        }
//        goldPosition = GoldPosition.LEFT;
    }

    public void hookDriveOff ( MecanumDrive myMechDrive, LiftMotor myLiftMotor) {

        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
      //  linearOp.sleep(sleepTime);
        //Change values if done so in MecanumMineralMinerCrater!
        myMechDrive.driveForward (SPD_DRIVE_HIGH, .3);                  // get away from the lander
      //  linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH, 1.2);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
     //   linearOp.sleep(sleepTime);

    }


    public void driveAwayFromHook (GyroCompetition myGyro, MecanumDrive myMechDrive) {

        myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.2);
       // linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(46, myMechDrive);
        myMechDrive.stopMotors();

        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .8);


     //   linearOp.sleep(sleepTime);

    }

//    public void releaseTM (IntakeRotaterServos myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeSpinnerMotor myIntakeSpinnerMotor) {
//        myIntakeRotator.loweredRotater();
//    //    linearOp.sleep(sleepTime);
//
//        myIntakeExtenderArm.extendIntakeArm(1);          //extend extender to knock off mineral
//     //   linearOp.sleep(2000);
//        myIntakeExtenderArm.stopIntakeArm();
//     //   linearOp.sleep(sleepTime);
//
//        myIntakeSpinnerMotor.intakeSpinner(-1);
//     //   linearOp.sleep(servoRotateTeamMarker);
//
//
//        myIntakeExtenderArm.retractIntactArm(1);              // retract extender
//    //    linearOp.sleep(2000);
//        myIntakeExtenderArm.stopIntakeArm();
//    //    linearOp.sleep(sleepTime);
//
//        myIntakeSpinnerMotor.stopMotors();
//     //   linearOp.sleep(sleepTime);
//
//        myIntakeRotator.raisedRotater();
//     //   linearOp.sleep(sleepTime);
//    }

    public void LowerReleaseTM(IntakeExtenderArm myIntakeExtenderArm, IntakeRotaterServos myIntakeRotator, IntakeSpinnerMotor myIntakeSpinnerMotor) {
        // extend arm and lower rotator
        // rotator will spin to release TM
        // extender will retract and rotator will raise

        myIntakeRotator.loweredRotater();
        linearOp.sleep(sleepTime);

        myIntakeExtenderArm.extendIntakeArm(1);          //extend extender to knock off mineral
        linearOp.sleep(2000);
        myIntakeExtenderArm.stopIntakeArm();
        linearOp.sleep(sleepTime);

        myIntakeSpinnerMotor.intakeSpinner(-1);
        linearOp.sleep(servoRotateTeamMarker);


        myIntakeExtenderArm.retractIntactArm(1);              // retract extender
        linearOp.sleep(2000);
        myIntakeExtenderArm.stopIntakeArm();
        linearOp.sleep(sleepTime);

        myIntakeSpinnerMotor.stopMotors();
        linearOp.sleep(sleepTime);

        myIntakeRotator.raisedRotater();
        linearOp.sleep(sleepTime);
    }


    public void goToStartPosition (GyroCompetition myGyro, MecanumDrive myMechDrive) {

        myMechDrive.strafeLeft(SPD_DRIVE_HIGH, .8);

        myGyro.gyroOrientMecanum(-5, myMechDrive);
        myMechDrive.stopMotors();
        //linearOp.sleep(sleepTime);

        myMechDrive.driveBackward(SPD_DRIVE_HIGH, 2.2);
        //linearOp.sleep(sleepTime);

        myGyro.gyroOrientMecanum(0, myMechDrive);
        myMechDrive.stopMotors();
        //linearOp.sleep(sleepTime);
    }


    public void collectMineral (GyroCompetition myGyro, MecanumDrive myMechDrive, IntakeExtenderArm myIntakeExtenderArm, IntakeSpinnerMotor myIntakeSpinnerMotor, IntakeRotaterServos myIntakeRotator) {


    }


    public void scoreMineral ( ) {

    }

























    public void codeExamples (GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotaterServos myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeSpinnerMotor myIntakeSpinnerMotor, MineralLift myMineralLift, LanderServo myLanderServo) {

        //GYRO TURNS
        //Use the following 3 lines of code for any Gyro Turns:
        myGyro.gyroOrientMecanum(-90, myMechDrive);    //Angle the robot turns.
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        //Robot Movement..
        //Reference the MecanumDrive class different classes.
        myMechDrive.driveForward (SPD_DRIVE_MED, .5);                  // get away from the lander
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .6);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);
        myMechDrive.driveBackward(SPD_DRIVE_HIGH, .51);
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


        //LANDER SERVO
        //Score minerals in lander
        myLanderServo.landerServoScore();
        //collect mineral in the sorting hat.  Default position unless need to score.
        myLanderServo.landerServoCollect();
    }
}
