package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotaterServos;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LanderServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.MineralLift;
import org.firstinspires.ftc.teamcode.robot.testing.mechanisms.Gyro;


public class MMM2ndAttemptCraterLanderScorer {              // hits depot and then does sample

    public MMM2ndAttemptCraterLanderScorer() {

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


    public void driveTowardDepot (GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotaterServos myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo) {
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
        linearOp.sleep(sleepTime);

        myMechDrive.driveForward(SPD_DRIVE_HIGH, .5);
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .6);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);

        myMechDrive.driveBackward(SPD_DRIVE_HIGH,.6);

        myGyro.gyroOrientMecanum(-5, myMechDrive);                //orients self with red tape so parallel to tape.
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .35);  // to get away from lander - too far = hit mineral, not enough = hit lander going to wall

        myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.5);


    }

    public void RotateDriveTowardDepot(GyroCompetition myGyro, MecanumDrive myMechDrive) {
        myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 1);              // fixing Gyro issue
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(46, myMechDrive);              // Orient for straight drive to depot
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);

        myMechDrive.setMotorPowerStrafeRight(SPD_DRIVE_HIGH);                      // Align to wall
        linearOp.sleep(800);                               // Time for straffing
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);
        myMechDrive.setMotorPowerStrafeLeft(SPD_DRIVE_HIGH); //make sure a little off wall so robot does not hit wall seam  was med
        linearOp.sleep(100);
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myGyro.gyroOrientMecanum(46, myMechDrive);              // Orient for straight drive to depot was 137
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);
        myMechDrive.driveForward(SPD_DRIVE_HIGH, .7);

    }

    public void LowerReleaseTM(IntakeExtenderArm myIntakeExtenderArm, IntakeRotaterServos myIntakeRotator, IntakeServo myIntakeServo) {
        // extend arm and lower rotator
        // rotator will spin to release TM
        // extender will retract and rotator will raise

        myIntakeRotator.loweredRotater();
        linearOp.sleep(sleepTime);
        myIntakeExtenderArm.extendIntakeArm(1000, SPD_DRIVE_HIGH, 2);
        linearOp.sleep(sleepTime);

        myIntakeServo.IntakeServoReverse();
        linearOp.sleep(servoRotateTeamMarker);

        myIntakeExtenderArm.retractIntakeArm(0, SPD_DRIVE_HIGH, 2);
        linearOp.sleep(sleepTime);

        myIntakeExtenderArm.retractPowerAuto(1);
        linearOp.sleep(1000);
        myIntakeExtenderArm.stopIntakeArm();
        myIntakeServo.stopIntakeServo();

        myIntakeRotator.raisedRotater();
        linearOp.sleep(sleepTime);
    }


    public void lineUpForScoring (MecanumDrive myMechDrive, Gyro myGyro) {
        myMechDrive.driveBackward(SPD_DRIVE_HIGH, -3);

        myMechDrive.rotateRight(SPD_DRIVE_HIGH, 1);
        myGyro.gyroOrientMecanum(-7, myMechDrive);
        myMechDrive.driveBackward(SPD_DRIVE_HIGH, -6);

        myMechDrive.rotateRight(SPD_DRIVE_HIGH,1);
        myGyro.gyroOrientMecanum(-90, myMechDrive);
    }


    public void sampleScoreGoldMineral (IntakeExtenderArm myIntakeExtenderArm, IntakeRotaterServos myIntakeRotator, IntakeServo myIntakeServo, MineralLift myMineralLift, LanderServo myLanderServo, MecanumDrive myMechDrive, Gyro myGyro) {
        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
            case LEFT:
                linearOp.telemetry.update();
                myMechDrive.rotateRight(SPD_DRIVE_HIGH, .5);          // fixing Gyro issue
                linearOp.sleep(sleepTime);

                linearOp.telemetry.update();
                myGyro.gyroOrientMecanum(-65, myMechDrive);           // different (34)
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);


                myIntakeServo.IntakeServoForward();                         //start spinner
                linearOp.sleep(sleepTime);

                myIntakeRotator.loweredRotater();                // lower rotater
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.extendIntakeArm(1000, SPD_ARM_MED, 2);          //extend extender to knock off mineral
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntakeArm(0,SPD_ARM_MED,2);              // retract extender
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);                                //second check to make sure extender is all the way in
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.raisedRotater();                                        //raise rotater
                linearOp.sleep(sleepTime);
                myIntakeServo.stopIntakeServo();


                break;

            case MIDDLE:
                myGyro.gyroOrientMecanum(-90, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeServo.IntakeServoForward();
                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.extendIntakeArm(1000, SPD_ARM_MED, 2);
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntakeArm(0,SPD_ARM_MED,2);
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.raisedRotater();
                linearOp.sleep(sleepTime);
                myIntakeServo.stopIntakeServo();
                linearOp.sleep(sleepTime);

                break;

            case RIGHT:
//                linearOp.telemetry.addData("RIGHT", goldPosition);
//                linearOp.telemetry.update();
                myMechDrive.rotateRight(SPD_DRIVE_HIGH, .5);         // fixing Gyro issue
                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(-120, myMechDrive);          // Gyro angles appears correct. was -14
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeServo.IntakeServoForward();
                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArm(1000, SPD_ARM_MED, 2);
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntakeArm(0,SPD_ARM_MED,2);
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.raisedRotater();
                linearOp.sleep(sleepTime);
                myIntakeServo.stopIntakeServo();
                break;
        }

        myIntakeExtenderArm.retractIntactArm(1);
        linearOp.sleep(100);
        myIntakeExtenderArm.stopIntakeArm();
        linearOp.sleep(sleepTime);

        myIntakeServo.IntakeServoReverseTime();
        linearOp.sleep(sleepTime);
        myLanderServo.releaseMinerals();
        linearOp.sleep(sleepTime);
        myIntakeServo.stopIntakeServo();
        linearOp.sleep(sleepTime);
        myLanderServo.keepMineralsIn();
        linearOp.sleep(sleepTime);

        myMineralLift.RaiseMineralLift();
        linearOp.sleep(sleepTime);

        myLanderServo.landerServoScore();
        linearOp.sleep(sleepTime);

        myLanderServo.landerServoCollect();
        linearOp.sleep(sleepTime);

        myMineralLift.LowerMineralLift();
        linearOp.sleep(sleepTime);

    }

    public void ScoreMineralsRepeat ( MineralLift myMineralLift, IntakeRotaterServos myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo, LanderServo myLanderServo) {

        while (AutoTime.time() < 20000) {

                myIntakeExtenderArm.extendIntakeArm(1000,SPD_DRIVE_HIGH, 2);
                linearOp.sleep(sleepTime);

                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);

                myIntakeServo.IntakeServoForward();

                myIntakeExtenderArm.extendIntakeArm(2100, SPD_DRIVE_HIGH, 3);
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
        myIntakeExtenderArm.extendIntakeArm(1000, SPD_DRIVE_HIGH, 3.5);
        linearOp.sleep(sleepTime);

        myIntakeRotator.loweredRotater();
        linearOp.sleep(sleepTime);

        myIntakeExtenderArm.extendIntakeArm(2000, SPD_DRIVE_HIGH, 2);


        }
    }