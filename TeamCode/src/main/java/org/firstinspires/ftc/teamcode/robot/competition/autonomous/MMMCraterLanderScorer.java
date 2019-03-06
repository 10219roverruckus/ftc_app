package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LanderServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.MineralLift;


public class MMMCraterLanderScorer {

    public MMMCraterLanderScorer() {

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

    public ElapsedTime AutoTime;


    // variables and constants used by color sensor

    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }


    // new methods for this class

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


    public void driveTowardDepot (GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo) {
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
        linearOp.sleep(sleepTime);

        myMechDrive.driveForward(SPD_DRIVE_HIGH, .5);
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .3);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);

        myGyro.gyroOrientMecanum(5, myMechDrive);

        myMechDrive.driveForward(SPD_DRIVE_HIGH, 2);
        myGyro.gyroOrientMecanum(45, myMechDrive);
        myIntakeExtenderArm.retractPowerAuto(1);

        myMechDrive.driveForward(SPD_DRIVE_HIGH, 1);


    }

    public void LowerReleaseTM(IntakeExtenderArm myIntakeExtenderArm, IntakeRotator myIntakeRotator, IntakeServo myIntakeServo) {
        // extend arm and lower rotator
        // rotator will spin to release TM
        // extender will retract and rotator will raise

        myIntakeRotator.mineralRotateLowerEncoder();
        linearOp.sleep(sleepTime);
        myIntakeExtenderArm.extendIntakeArmAuto();
        linearOp.sleep(sleepTime);

        myIntakeServo.IntakeServoReverse();
        linearOp.sleep(servoRotateTeamMarker);

        myIntakeExtenderArm.retractIntakeArmAuto();
        linearOp.sleep(sleepTime);

        myIntakeExtenderArm.retractPowerAuto(1);
        linearOp.sleep(1000);
        myIntakeExtenderArm.stopIntakeArm();
        myIntakeServo.stopIntakeServo();

        myIntakeRotator.mineralRotateRaiseEncoder();
        linearOp.sleep(sleepTime);
    }

    public void setUpForLander (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDistance) {

        myMechDrive.driveBackward(SPD_DRIVE_HIGH,1);                                  // drive backward toward lander
        myGyro.gyroOrientMecanum(45, myMechDrive);                                      // angle toward going inbetween alnder and minerals

        myMechDrive.driveBackward(SPD_DRIVE_HIGH, 2);                                // drive back through the lander and minerals
        myMechDrive.rotateRight(SPD_DRIVE_HIGH,.4);                                 // rotate to face minerals

        myGyro.gyroOrientMecanum(90, myMechDrive);                                      //  fix angle

        Color.RGBToHSV((int) (myRevColorDistance.revColorSensor.red() * SCALE_FACTOR),                  // checking for red line to line up with the lander
                (int) (myRevColorDistance.revColorSensor.green() * SCALE_FACTOR),
                (int) (myRevColorDistance.revColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        while (hsvValues[0] > RED_THRESHOLD && hsvValues[0] < BLUE_THRESHOLD) {
            Color.RGBToHSV((int) (myRevColorDistance.revColorSensor.red() * SCALE_FACTOR),
                    (int) (myRevColorDistance.revColorSensor.green() * SCALE_FACTOR),
                    (int) (myRevColorDistance.revColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            myMechDrive.setMotorSpeeds(SPD_DRIVE_MED);

            linearOp.idle();
        }


    }

    public void intakeGoldMineral (GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo) {
        switch (goldPosition) {
            case LEFT:
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, .5);          // fixing Gyro issue
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(60, myMechDrive);              //rotate toward gold mineral
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeServo.IntakeServoForward();                             //start intake servos to intake mineral
                myIntakeRotator.mineralRotateLowerEncoder();                    //drop rotater
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.extendIntakeArmAuto();                      //extend extender
                linearOp.sleep(sleepTime);
                myIntakeExtenderArm.retractIntakeArmAuto();                     //retract extender
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);            //second check to make sure extender is in
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.mineralRotateRaiseEncoder();                   //raise rotater with mineral in the intake
                linearOp.sleep(sleepTime);
                myIntakeServo.stopIntakeServo();                               // stop spinners
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 0.7);              // fixing Gyro issue of over rotating
                break;


            case MIDDLE:

                myGyro.gyroOrientMecanum(90, myMechDrive);             //rotate toward gold mineral
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeServo.IntakeServoForward();                            //start intake servos to intake mineral
                myIntakeRotator.mineralRotateLowerEncoder();                    //drop rotater
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.extendTowardMiddleMineral();            //extend extender
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntakeArmAuto();                 //retract extender
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);            //second check for retracting extender
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.mineralRotateRaiseEncoder();                //raise rotater
                linearOp.sleep(sleepTime);
                myIntakeServo.stopIntakeServo();                            //stop spinners
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 1.3);              // fixing Gyro issue of over rotating
                linearOp.sleep(sleepTime);
                break;

            case RIGHT:
                myMechDrive.rotateRight(SPD_DRIVE_HIGH, .5);         // fixing Gyro issue
                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(110, myMechDrive);          // Gyro angles appears correct. was -14
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeServo.IntakeServoForward();
                myIntakeRotator.mineralRotateLowerEncoder();                //drop rotater
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArmAuto();                  //extend extender
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntakeArmAuto();                 //retract extender
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);        //second check to make sure extender is in
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.mineralRotateRaiseEncoder();                //raise rotater
                linearOp.sleep(sleepTime);
                myIntakeServo.stopIntakeServo();
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 1.9);              // fixing Gyro issue of over rotating was 1.9
                break;
        }
    }


    public void ScoreGoldMineral ( MineralLift myMineralLift, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo, LanderServo myLanderServo) {
        myIntakeExtenderArm.retractPowerAuto(1);                // getting ready to transfer gold mineral
        linearOp.sleep(1000);
        myIntakeExtenderArm.stopIntakeArm();

        myIntakeServo.IntakeServoReverseTime();
        linearOp.sleep(sleepTime);
        myLanderServo.releaseMinerals();                                    //transfer into mineral lift
        linearOp.sleep(sleepTime);

        myMineralLift.extendLiftMotorFullyEncoders();                       //extend Mineral lift with gold mineral
        linearOp.sleep(sleepTime);

        myLanderServo.landerServoScore();                                   //score mineral
        linearOp.sleep(sleepTime);

        myMineralLift.retractLiftMotorFullyEncoders();                      //retract Mineral Lift
        linearOp.sleep(sleepTime);

    }

    public void ScoreMineralsRepeat ( MineralLift myMineralLift, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo, LanderServo myLanderServo) {

        while (AutoTime.time() < 20000) {

            if (AutoTime.time() < 20000) {
                myIntakeExtenderArm.extendIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateLowerEncoder();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArmAllTheWay();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);                // getting ready to transfer gold mineral
                linearOp.sleep(sleepTime);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeServo.IntakeServoReverseTime();
                linearOp.sleep(sleepTime);
                myLanderServo.releaseMinerals();                                    //transfer into mineral lift
                linearOp.sleep(sleepTime);

                myMineralLift.extendLiftMotorFullyEncoders();                       //extend Mineral lift with gold mineral
                linearOp.sleep(sleepTime);

                myLanderServo.landerServoScore();                                   //score mineral
                linearOp.sleep(sleepTime);

                myMineralLift.retractLiftMotorFullyEncoders();                      //retract Mineral Lift
                linearOp.sleep(sleepTime);

            }
            else {
                myIntakeExtenderArm.extendIntakeArmAllTheWay();
                linearOp.sleep(sleepTime);
            }

        }
    }


}
