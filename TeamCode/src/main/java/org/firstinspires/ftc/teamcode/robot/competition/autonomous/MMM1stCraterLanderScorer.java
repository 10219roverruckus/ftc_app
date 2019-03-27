package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotaterServos;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeSpinnerMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LanderServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.MineralLift;
import org.firstinspires.ftc.teamcode.robot.testing.mechanisms.Gyro;


public class MMM1stCraterLanderScorer {

    public MMM1stCraterLanderScorer() {

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


    public void driveTowardDepot (GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotaterServos myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeSpinnerMotor myIntakeSpinnerMotor) {
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
        linearOp.sleep(sleepTime);

        myMechDrive.driveForward(SPD_DRIVE_HIGH, .5);
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .6);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);

        myMechDrive.driveBackward(SPD_DRIVE_HIGH,.6);

        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
            case LEFT:

                linearOp.telemetry.update();
                myMechDrive.rotateRight(SPD_DRIVE_HIGH, .5);          // fixing Gyro issue
                linearOp.sleep(sleepTime);

                linearOp.telemetry.update();
                myGyro.gyroOrientMecanum(-65, myMechDrive);           // different (34)
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);


                myIntakeSpinnerMotor.intakeSpinner(1);                         //start spinner
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
                myIntakeSpinnerMotor.stopMotors();
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 0.7);              // fixing Gyro issue of over rotating

                break;

            case MIDDLE:
                myGyro.gyroOrientMecanum(-90, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeSpinnerMotor.intakeSpinner(1);
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
                myIntakeSpinnerMotor.stopMotors();
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 1.3);              // fixing Gyro issue of over rotating
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
                myIntakeSpinnerMotor.intakeSpinner(1);
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
                myIntakeSpinnerMotor.stopMotors();
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 1.9);              // fixing Gyro issue of over rotating was 1.9
                break;
        }


    }

    public void RotateDriveWall(GyroCompetition myGyro, MecanumDrive myMechDrive, IntakeExtenderArm myIntakeExtenderArm) {

        myGyro.gyroOrientMecanum(-5, myMechDrive);                //orients self with red tape so parallel to tape.
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myMechDrive.strafeRight(SPD_DRIVE_MED, .55);  // to get away from lander - too far = hit mineral, not enough = hit lander going to wall

        switch (goldPosition) {
            case LEFT:
                myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.2);
                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();
                break;
            case MIDDLE:
                myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.3);
                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();
                break;
            case RIGHT:
                myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.4);
                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();
                break;
        }


        linearOp.sleep(sleepTime);
    }

    public void RotateDriveTowardDepot(GyroCompetition myGyro, MecanumDrive myMechDrive) {
        myMechDrive.rotateLeft(SPD_DRIVE_MED, 1);              // fixing Gyro issue
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

    public void LowerReleaseTM(IntakeExtenderArm myIntakeExtenderArm, IntakeRotaterServos myIntakeRotator, IntakeSpinnerMotor myIntakeSpinnerMotor) {
        // extend arm and lower rotator
        // rotator will spin to release TM
        // extender will retract and rotator will raise

        myIntakeRotator.loweredRotater();
        linearOp.sleep(sleepTime);
        myIntakeExtenderArm.extendIntakeArm(1000, SPD_ARM_MED, 2);
        linearOp.sleep(sleepTime);

        myIntakeSpinnerMotor.intakeSpinner(-1);
        linearOp.sleep(servoRotateTeamMarker);

        myIntakeExtenderArm.retractIntakeArm(0, SPD_ARM_MED, 2);
        linearOp.sleep(sleepTime);

        myIntakeExtenderArm.retractPowerAuto(1);
        linearOp.sleep(1000);
        myIntakeExtenderArm.stopIntakeArm();
        myIntakeSpinnerMotor.stopMotors();

        myIntakeRotator.raisedRotater();
        linearOp.sleep(sleepTime);
    }


    public void lineUpForScoring (MecanumDrive myMechDrive, Gyro myGyro) {
        myMechDrive.driveBackward(SPD_DRIVE_HIGH, -3);

        myMechDrive.rotateRight(SPD_DRIVE_HIGH, 1);
        myGyro.gyroOrientMecanum(-7, myMechDrive);
        myMechDrive.driveBackward(SPD_DRIVE_HIGH, -6);

        myGyro.gyroOrientMecanum(-90, myMechDrive);
    }

    public void ScoreGoldMineral ( MineralLift myMineralLift, IntakeExtenderArm myIntakeExtenderArm, IntakeSpinnerMotor myIntakeSpinnerMotor, LanderServo myLanderServo, IntakeRotaterServos myIntakeRotater) {

        myIntakeExtenderArm.extendIntakeArm(2000, SPD_DRIVE_HIGH, 2);
        linearOp.sleep(sleepTime);

        myIntakeRotater.loweredRotater();

        myIntakeSpinnerMotor.intakeSpinner(1);

        myIntakeRotater.raisedRotater();

        myIntakeExtenderArm.retractIntakeArm(0, SPD_DRIVE_HIGH, 2);

        myIntakeExtenderArm.retractPowerAuto(1);                // getting ready to transfer gold mineral
        linearOp.sleep(1000);
        myIntakeExtenderArm.stopIntakeArm();


        linearOp.sleep(sleepTime);
//        myLanderServo.releaseMinerals();                                    //transfer into mineral lift
        linearOp.sleep(sleepTime);

        myMineralLift.extendLiftMotorFullyEncoders();                       //extend Mineral lift with gold mineral
        linearOp.sleep(sleepTime);

        myLanderServo.landerServoScore();                                   //score mineral
        linearOp.sleep(sleepTime);

        myLanderServo.landerServoCollect();

        myMineralLift.retractLiftMotorFullyEncoders();                      //retract Mineral Lift
        linearOp.sleep(sleepTime);

    }

    public void ScoreMineralsRepeat ( MineralLift myMineralLift, IntakeRotaterServos myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeSpinnerMotor myIntakeSpinnerMotor, LanderServo myLanderServo) {

        while (AutoTime.time() < 20000) {

                myIntakeExtenderArm.extendIntakeArm(1000,SPD_DRIVE_HIGH, 2);
                linearOp.sleep(sleepTime);

                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);

            myIntakeSpinnerMotor.intakeSpinner(1);

                myIntakeExtenderArm.extendIntakeArm(2100, SPD_DRIVE_HIGH, 3);
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);                // getting ready to transfer gold mineral
                linearOp.sleep(sleepTime);
                myIntakeExtenderArm.stopIntakeArm();

            myIntakeSpinnerMotor.intakeSpinner(-1);
                linearOp.sleep(sleepTime);

//                myLanderServo.releaseMinerals();                                    //transfer into mineral lift
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

        myIntakeExtenderArm.extendIntakeArm(2000, SPD_ARM_MED, 2);


        }
    }



