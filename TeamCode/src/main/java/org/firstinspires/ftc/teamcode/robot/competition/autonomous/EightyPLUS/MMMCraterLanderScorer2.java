package org.firstinspires.ftc.teamcode.robot.competition.autonomous.EightyPLUS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    final long sleepTime = 50;
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

        myIntakeExtender.retractIntactArm(1);
        linearOp.sleep(100);
        myIntakeExtender.stopIntakeArm();

        myMechDrive.rotateLeft(SPD_DRIVE_HIGH, .5);
        linearOp.sleep(sleepTime);

        myGyro.gyroOrientMecanum(42, myMechDrive);      // was 46
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myMechDrive.strafeRight(SPD_DRIVE_MED, 1);      // .3
        linearOp.sleep(sleepTime);

    }


    public void LowerReleaseTM(IntakeExtenderArm myIntakeExtenderArm, IntakeRotaterServos myIntakeRotator, IntakeSpinnerMotor myIntakeSpinnerMotor, MecanumDrive myMechDrive) {
        // extend arm and lower rotator
        // rotator will spin to release TM
        // extender will retract and rotator will raise

        myMechDrive.driveForward(SPD_DRIVE_MED, .3);
        linearOp.sleep(sleepTime);

        myIntakeSpinnerMotor.intakeSpinner(1);

        myIntakeRotator.loweredRotater();
        linearOp.sleep(sleepTime);

        myIntakeExtenderArm.extendIntakeArm(1);          //extend extender to knock off mineral
        linearOp.sleep(1200);                   // was 1400
        myIntakeExtenderArm.stopIntakeArm();
        linearOp.sleep(sleepTime);

        myIntakeSpinnerMotor.intakeSpinner(-1);

        myIntakeExtenderArm.retractIntactArm(1);              // retract extender
        linearOp.sleep(1250);               // was 1450
        myIntakeExtenderArm.stopIntakeArm();
        linearOp.sleep(sleepTime);

        myIntakeSpinnerMotor.stopMotors();
        linearOp.sleep(sleepTime);

        myIntakeRotator.raisedRotater();
        linearOp.sleep(sleepTime);

        myMechDrive.driveBackward(SPD_DRIVE_MED, .3);
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
                myMechDrive.driveBackward(SPD_DRIVE_HIGH, 1.6);     // was 2.2
                linearOp.sleep(sleepTime);

                myMechDrive.rotateRight(SPD_DRIVE_HIGH, .5);
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-59, myMechDrive);     // was -58
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntactArm(1);
                linearOp.sleep(300);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);

                myIntakeSpinnerMotor.intakeSpinner(1);

                myMechDrive.driveForward(SPD_DRIVE_HIGH, .65);       // was .8
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-81, myMechDrive);         // was -76
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-81, myMechDrive);     //-76
                myMechDrive.stopMotors();
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

                myGyro.gyroOrientMecanum(-98, myMechDrive);                 // was -98 missed mineral
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

             break;



            case RIGHT:

                myMechDrive.driveBackward(SPD_DRIVE_HIGH, 2.5);
                linearOp.sleep(sleepTime);

                myMechDrive.rotateRight(SPD_DRIVE_HIGH, 1.5);
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-112, myMechDrive);            // was -112  was about to mis mineral
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

                myMechDrive.driveBackward(SPD_DRIVE_HIGH, .15);         // not break the plane of the crater was .25

                myGyro.gyroOrientMecanum(-112, myMechDrive);            // was -114
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
             break;

        }

        myIntakeExtenderArm.retractIntactArm(1);

        myIntakeRotator.raisedRotater();
        linearOp.sleep(400);

        myIntakeSpinnerMotor.intakeSpinner(-.75);                         // mineral is getting caught in intake____ was 700
        linearOp.sleep(900);

        myIntakeExtenderArm.stopIntakeArm();
        myIntakeSpinnerMotor.stopMotors();
    }



    public void scoreMineral (LanderServo myLanderServo, MineralLift myMineralLift, RevColorDistance myRevColorDistance) {

        myLanderServo.landerServoTravel();
        linearOp.sleep(100);

        myMineralLift.RaiseMineralLift();
        linearOp.sleep(1500);

        myLanderServo.landerServoScore();
        linearOp.sleep(700);

        myLanderServo.landerServoCollect();
        linearOp.sleep(200);


        while (myRevColorDistance.checkSensorMineralLift() == false && linearOp.opModeIsActive()) {
            myMineralLift.LowerMineralLift();
        }
        myMineralLift.stopMotors();
        linearOp.sleep(sleepTime);


    }

    public void additionalMineralScoring (IntakeExtenderArm myIntakeExtenderArm, IntakeRotaterServos myIntakeRotater, IntakeSpinnerMotor myIntakeSpinnerMotor, LanderServo myLanderServo, MineralLift myMineralLift, RevColorDistance myRevColorDistance, GyroCompetition myGyro, MecanumDrive myMechDrive) {


        switch (goldPosition) {
            case LEFT:

                myGyro.gyroOrientMecanum(-81, myMechDrive);     //-76
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArm(1);
                linearOp.sleep(300);
                myIntakeExtenderArm.stopIntakeArm();
//                linearOp.sleep(sleepTime);

//                linearOp.sleep(500);

                myIntakeSpinnerMotor.intakeSpinner(1);

                myIntakeRotater.loweredRotater();
                linearOp.sleep(200);

                myIntakeExtenderArm.extendIntakeArm(1);
                linearOp.sleep(400);
                myIntakeExtenderArm.stopIntakeArm();
                linearOp.sleep(200);
                myIntakeExtenderArm.retractIntactArm(1);
                linearOp.sleep(200);

                myIntakeRotater.raisedRotater();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntactArm(1);
                linearOp.sleep(550);
//                myIntakeExtenderArm.stopIntakeArm();
//                linearOp.sleep(sleepTime);


                break;
            case MIDDLE:

//                myGyro.gyroOrientMecanum(-98, myMechDrive);             // was -95        // affected scoring (almost missed mineral)
//                myMechDrive.stopMotors();
//                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArm(1);
                linearOp.sleep(150);                // was 300
                myIntakeExtenderArm.stopIntakeArm();
//                linearOp.sleep(sleepTime);

//                linearOp.sleep(500);

                myIntakeSpinnerMotor.intakeSpinner(1);

                myIntakeRotater.loweredRotater();
                linearOp.sleep(200);

                myIntakeExtenderArm.extendIntakeArm(1);             // was 400
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();
                linearOp.sleep(200);
                myIntakeExtenderArm.retractIntactArm(1);
                linearOp.sleep(200);

                myIntakeRotater.raisedRotater();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntactArm(1);
                linearOp.sleep(550);
//                myIntakeExtenderArm.stopIntakeArm();
//                linearOp.sleep(sleepTime);

                break;
            case RIGHT:

                myGyro.gyroOrientMecanum(-105, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeSpinnerMotor.intakeSpinner(1);

                myIntakeExtenderArm.extendIntakeArm(1);
                linearOp.sleep(300);
                myIntakeExtenderArm.stopIntakeArm();
//                linearOp.sleep(sleepTime);

//                linearOp.sleep(500);

                myIntakeRotater.loweredRotater();
                linearOp.sleep(200);

                myIntakeExtenderArm.extendIntakeArm(1);
                linearOp.sleep(400);
                myIntakeExtenderArm.stopIntakeArm();
                linearOp.sleep(200);
                myIntakeExtenderArm.retractIntactArm(1);
                linearOp.sleep(200);

                myIntakeRotater.raisedRotater();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntactArm(1);
                linearOp.sleep(550);
//                myIntakeExtenderArm.stopIntakeArm();
//                linearOp.sleep(sleepTime);

//                myIntakeSpinnerMotor.stopMotors();

                //SECOND ANGLE CHANGE THAT WAS OVER CORRECTING THE ANGLE
                myGyro.gyroOrientMecanum(-115, myMechDrive);            // was -114
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                break;
        }

        myIntakeSpinnerMotor.intakeSpinner(-.75);
        linearOp.sleep(900);
        myIntakeSpinnerMotor.stopMotors();
        linearOp.sleep(200);

        myIntakeExtenderArm.stopIntakeArm();

        myLanderServo.landerServoTravel();
        linearOp.sleep(200);


        myMineralLift.RaiseMineralLift();
        linearOp.sleep(1500);

        myLanderServo.landerServoScore();
        linearOp.sleep(900);

        myLanderServo.landerServoCollect();
        linearOp.sleep(200);


        while (myRevColorDistance.checkSensorMineralLift() == false && linearOp.opModeIsActive()) {
            myMineralLift.LowerMineralLift();
        }
        myMineralLift.stopMotors();
        linearOp.sleep(sleepTime);


        myIntakeExtenderArm.extendIntakeArm(1);            // extending to grab second cycle
        linearOp.sleep(400);
        myIntakeExtenderArm.stopIntakeArm();
        linearOp.sleep(sleepTime);

        myIntakeRotater.loweredRotater();

        myIntakeSpinnerMotor.intakeSpinner(1);
        myIntakeExtenderArm.extendIntakeArm(1);
        linearOp.sleep(300);
        myIntakeExtenderArm.stopIntakeArm();

    }
}
