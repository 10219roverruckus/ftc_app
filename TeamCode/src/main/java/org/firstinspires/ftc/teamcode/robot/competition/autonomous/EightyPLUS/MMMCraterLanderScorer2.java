package org.firstinspires.ftc.teamcode.robot.competition.autonomous.EightyPLUS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.autonomous.GoldPosition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotaterServos;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeSpinnerMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LanderServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.MineralLift;
import org.firstinspires.ftc.teamcode.robot.testing.mechanisms.Gyro;


public class MMMCraterLanderScorer2 {

    public MMMCraterLanderScorer2() {

    }

    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;
    public final int RED_THRESHOLD = 30;                //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100;              //maybe a higher threshold (12.5)
    float hsvValues[] = {0F, 0F, 0F};

    // created constant variables that are used for speed (different setting)

    final double SPD_DRIVE_LOW = .38;                  //Lowest speed
    final double SPD_DRIVE_MED = 0.5;   //was .6 but for testing we slowed it down
    final double SPD_DRIVE_HIGH = .75; // was .8 but for testing we slowed it down
    final double SPD_DRIVE_MAX = 1.0;
    final double SPD_ARM_MED = .5;
    final long sleepTime = 100;
    final int servoRotateTeamMarker = 1000;


    // variables and constants used by color sensor

    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }


    // new methods for this class

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

        myLiftMotor.extendLiftMotorFullyEncoders();// using encoders rather than distance sensor

        myMechDrive.driveForward (SPD_DRIVE_HIGH, .3);                  // get away from the lander
        linearOp.sleep(sleepTime);

        myMechDrive.strafeRight(SPD_DRIVE_MED, 1.5);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);

    }


    public void driveAwayFromHook (GyroCompetition myGyro, MecanumDrive myMechDrive, IntakeExtenderArm myIntakeExtender) {

        myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.2);
        linearOp.sleep(sleepTime);

        myIntakeExtender.retractIntactArm(SPD_ARM_MED);
        linearOp.sleep(300);
        myIntakeExtender.stopIntakeArm();

        myGyro.gyroOrientMecanum(42, myMechDrive);      // was 46
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myMechDrive.strafeRight(SPD_DRIVE_MED, 1);      // .3
        linearOp.sleep(sleepTime);

    }


    public void LowerReleaseTM(IntakeExtenderArm myIntakeExtenderArm, IntakeRotaterServos myIntakeRotator, IntakeSpinnerMotor myIntakeSpinnerMotor) {
        // extend arm and lower rotator
        // rotator will spin to release TM
        // extender will retract and rotator will raise

        myIntakeSpinnerMotor.intakeSpinner(1);

        myIntakeRotator.loweredRotater();
        linearOp.sleep(sleepTime);

        myIntakeExtenderArm.extendIntakeArm(1);          //extend extender to knock off mineral
        linearOp.sleep(2000);
        myIntakeExtenderArm.stopIntakeArm();
        linearOp.sleep(sleepTime);

        myIntakeSpinnerMotor.intakeSpinner(-1);

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

        myMechDrive.strafeLeft(SPD_DRIVE_HIGH, 1.2);    // was .8

        myGyro.gyroOrientMecanum(-8, myMechDrive);      // was -5
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);


    }


    public void knockOffMineral (GyroCompetition myGyro, MecanumDrive myMechDrive, IntakeRotaterServos myIntakeRotator, IntakeSpinnerMotor myIntakeSpinnerMotor, IntakeExtenderArm myIntakeExtenderArm) {

        switch (goldPosition) {
            case LEFT:
                myMechDrive.driveBackward(SPD_DRIVE_HIGH, 2.2);
                linearOp.sleep(sleepTime);

                myMechDrive.rotateRight(SPD_DRIVE_HIGH, .5);
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-45, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntactArm(1);
                linearOp.sleep(300);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);

                myIntakeSpinnerMotor.intakeSpinner(1);


                myMechDrive.driveForward(SPD_DRIVE_HIGH, 1);
                linearOp.sleep(sleepTime);



                break;

            case MIDDLE:

                myMechDrive.driveBackward(SPD_DRIVE_HIGH, 2.2);
                linearOp.sleep(sleepTime);

                myMechDrive.rotateRight(SPD_DRIVE_HIGH, 1);
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-80, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntactArm(1);
                linearOp.sleep(300);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);

                myIntakeSpinnerMotor.intakeSpinner(1);


                myMechDrive.driveForward(SPD_DRIVE_HIGH, 1);
                linearOp.sleep(sleepTime);
             break;

            case RIGHT:

                myMechDrive.driveBackward(SPD_DRIVE_HIGH, 2.42);
                linearOp.sleep(sleepTime);

                myMechDrive.rotateRight(SPD_DRIVE_HIGH, 1.5);
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-113, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntactArm(1);
                linearOp.sleep(300);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);

                myIntakeSpinnerMotor.intakeSpinner(1);


                myMechDrive.driveForward(SPD_DRIVE_HIGH, .95);
                linearOp.sleep(300);

                myGyro.gyroOrientMecanum(-114, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
             break;

        }

        myIntakeExtenderArm.retractIntactArm(1);

        myIntakeRotator.raisedRotater();
        linearOp.sleep(400);

        myIntakeSpinnerMotor.intakeSpinner(-1);
        linearOp.sleep(500);

        myIntakeExtenderArm.stopIntakeArm();
        myIntakeSpinnerMotor.stopMotors();
    }



    public void scoreMineral (LanderServo myLanderServo, MineralLift myMineralLift, RevColorDistance myRevColorDistance) {

        myLanderServo.landerServoTravel();
        linearOp.sleep(100);

        myMineralLift.RaiseMineralLift();
        linearOp.sleep(1200);

        myLanderServo.landerServoScore();
        linearOp.sleep(500);

        myLanderServo.landerServoCollect();
        linearOp.sleep(200);

        while (myRevColorDistance.checkSensorMineralLift() == false) {
            myMineralLift.LowerMineralLift();
        }
        myMineralLift.stopMotors();
        linearOp.sleep(sleepTime);


    }

    public void additionalMineralScoring (Gyro myGyro, IntakeExtenderArm myIntakeExtenderArm, MecanumDrive myMechDrive) {

        myGyro.gyroOrientMecanum(-111, myMechDrive);
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myIntakeExtenderArm.extendIntakeArm(1);
        linearOp.sleep(700);
        myIntakeExtenderArm.stopIntakeArm();

    }












}
