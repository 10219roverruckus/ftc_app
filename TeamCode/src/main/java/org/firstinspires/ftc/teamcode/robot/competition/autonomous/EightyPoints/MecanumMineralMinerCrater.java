package org.firstinspires.ftc.teamcode.robot.competition.autonomous.EightyPoints;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.autonomous.GoldPosition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotaterServos;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeSpinnerMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;


public class MecanumMineralMinerCrater {

    public MecanumMineralMinerCrater() {

    }

    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;
    public final int RED_THRESHOLD = 30;                //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100;              //maybe a higher threshold (12.5)
    float hsvValues[] = {0F, 0F, 0F};

    // created constant variables that are used for speed (different setting)

    final double SPD_DRIVE_LOW = .38;                  //Lowest speed
    final double SPD_DRIVE_MED = 0.5;   //was .6 but for testing we slowed it down
    final double SPD_DRIVE_HIGH = .70; // was .8 but for testing we slowed it down
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

    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotaterServos myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeSpinnerMotor myIntakeSpinnerMotor) {

        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
        linearOp.sleep(sleepTime);

        myMechDrive.driveForward (SPD_DRIVE_MED, .3);                  // get away from the lander
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .6);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);
        myMechDrive.driveBackward(SPD_DRIVE_HIGH, .51);
        linearOp.sleep(sleepTime);

        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
            case LEFT:

                linearOp.telemetry.update();
                myMechDrive.rotateRight(SPD_DRIVE_HIGH, .5);          // fixing Gyro issue
                linearOp.sleep(sleepTime);

                linearOp.telemetry.update();
                myGyro.gyroOrientMecanum(-45, myMechDrive);           // different (34)
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);


                myIntakeSpinnerMotor.intakeSpinner(1);              //start spinner
                linearOp.sleep(sleepTime);

                myIntakeRotator.loweredRotater();                           // lower rotater - uses Servos
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.extendIntakeArm(1);          //extend extender to knock off mineral
                linearOp.sleep(1000);
                myIntakeExtenderArm.stopIntakeArm();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntactArm(1);              // retract extender
                linearOp.sleep(1000);
                myIntakeExtenderArm.stopIntakeArm();


                myIntakeRotator.raisedRotater();                                   //raise rotater
                linearOp.sleep(sleepTime);
                myIntakeSpinnerMotor.stopMotors();
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 0.7);              // fixing Gyro issue of over rotating

                break;

            case MIDDLE:
                myGyro.gyroOrientMecanum(-80, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myIntakeSpinnerMotor.intakeSpinner(1);
                linearOp.sleep(sleepTime);

                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.extendIntakeArm(1);          //extend extender to knock off mineral
                linearOp.sleep(700);
                myIntakeExtenderArm.stopIntakeArm();


                myIntakeExtenderArm.retractIntactArm(1);              // retract extender
                linearOp.sleep(700);
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
                myMechDrive.rotateRight(SPD_DRIVE_HIGH, 1.2);         // fixing Gyro issue
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-110, myMechDrive);          // Gyro angles appears correct. was -14
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myIntakeSpinnerMotor.intakeSpinner(1);
                linearOp.sleep(sleepTime);

                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArm(1);          //extend extender to knock off mineral
                linearOp.sleep(800);
                myIntakeExtenderArm.stopIntakeArm();


                myIntakeExtenderArm.retractIntactArm(1);              // retract extender
                linearOp.sleep(800);
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

        myMechDrive.strafeRight(SPD_DRIVE_MED, .65);  // to get away from lander - too far = hit mineral, not enough = hit lander going to wall

        switch (goldPosition) {
            case LEFT:
                myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.2);

//                myIntakeExtenderArm.retractPowerAuto(1);
//                linearOp.sleep(500);
//                myIntakeExtenderArm.stopIntakeArm();
                break;
            case MIDDLE:
                myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.3);
//                myIntakeExtenderArm.retractPowerAuto(1);
//                linearOp.sleep(500);
//                myIntakeExtenderArm.stopIntakeArm();
                break;
            case RIGHT:
                myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.4);
//                myIntakeExtenderArm.retractPowerAuto(1);
//                linearOp.sleep(500);
//                myIntakeExtenderArm.stopIntakeArm();
                break;
        }


        linearOp.sleep(sleepTime);
    }


    public void RotateDriveTowardDepot(GyroCompetition myGyro, MecanumDrive myMechDrive, IntakeExtenderArm myIntakeExtenderArm) {

        myIntakeExtenderArm.retractIntactArm(1);
        linearOp.sleep(100);
        myIntakeExtenderArm.stopIntakeArm();

        myMechDrive.rotateLeft(SPD_DRIVE_MED, .6);              // fixing Gyro issue
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(46, myMechDrive);              // Orient for straight drive to depot
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);

        myGyro.gyroOrientMecanum(46, myMechDrive);              // Orient for straight drive to depot
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);

        myMechDrive.setMotorPowerStrafeRight(SPD_DRIVE_HIGH);                      // Align to wall
        linearOp.sleep(700);                               // Time for straffing
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);
        myMechDrive.setMotorPowerStrafeLeft(SPD_DRIVE_HIGH); //make sure a little off wall so robot does not hit wall seam  was med
        linearOp.sleep(100);
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myMechDrive.driveForward(SPD_DRIVE_MED, 1.3);           // goal is to have the 0 line up with the seam on the field
    //Commenting out Gyro Correction since use wall to orient ourselves.
//        myGyro.gyroOrientMecanum(46, myMechDrive);              // Orient for straight drive to depot was 137
//        myMechDrive.stopMotors();                                      // Stop motors
//        linearOp.sleep(sleepTime);
//        myMechDrive.driveForward(SPD_DRIVE_HIGH, .7);


    }


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


    public void DriveParkInCrater(MecanumDrive myMechDrive) {
        // drive backward and park in crater

        myMechDrive.driveBackward(SPD_DRIVE_HIGH, 1.5);      // rotations was .2 // Drive backwards pass the seam
        linearOp.sleep(sleepTime);

//        myMechDrive.setMotorPowerStrafeRight(SPD_DRIVE_HIGH);       // was med
//        linearOp.sleep(100);
//        myMechDrive.stopMotors();

        myMechDrive.setMotorPowerStrafeRight(SPD_DRIVE_MED);
        linearOp.sleep(200);
        myMechDrive.stopMotors();


        myMechDrive.driveBackward(.28, .4);       //rotations was .4      // go slowly onto the crater without rolling off
        linearOp.sleep(sleepTime);
    }







    //BLAKE'S
//    public void CraterExtendArm(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo) {
//        myGyro.gyroReset();
//        myMechDrive.strafeLeft(SPD_DRIVE_HIGH,0.6);
//        linearOp.sleep(sleepTime);
//        myMechDrive.driveBackward(SPD_DRIVE_HIGH, 0.4);
//        linearOp.sleep(sleepTime);
//        myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 3.1);
//        linearOp.sleep(sleepTime);
//        myMechDrive.driveForward(SPD_DRIVE_HIGH, 0.5);
//        linearOp.sleep(sleepTime);
//        myMechDrive.strafeRightCurvedToCrater(0.5, 0.45, 0.7, 0.75, 2.2);
//        myIntakeExtenderArm.extendHalfIntakeArmAuto();
//        myIntakeServo.IntakeServoForward();
//        myIntakeRotator.mineralRotateLowerEncoder();
//        myIntakeExtenderArm.extendHalfIntakeArmAuto();
//        linearOp.sleep(1000);
//    }
}
