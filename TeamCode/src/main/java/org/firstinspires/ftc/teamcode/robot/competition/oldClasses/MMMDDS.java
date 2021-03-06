package org.firstinspires.ftc.teamcode.robot.competition.oldClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.autonomous.GoldPosition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotaterServos;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeSpinnerMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;


public class MMMDDS {

    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;
    public final int RED_THRESHOLD = 30;                //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100;              //maybe a higher threshold (12.5)
    float hsvValues[] = {0F, 0F, 0F};

    // created constant variables that are used for speed (different setting)

    final double SPD_DRIVE_LOW = .20;                  //Lowest speed
    final double SPD_DRIVE_MED = .4;                   //Default is  SPD_MED
    final double SPD_DRIVE_HIGH = .75;
    final double SPD_DRIVE_MAX = 1.0;
    final double SPD_ARM_MED = .5;
    final long sleepTime = 100;

    // variables and constants used by color sensor

    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }

    //in future, will pass Camera as Parameter
    public MMMDDS() {
    }


    // new Methods


    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive, IntakeRotaterServos myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeSpinnerMotor myIntakeSpinnerMotor) {

        linearOp.telemetry.addData("MINERAL", goldPosition);
        linearOp.telemetry.update();
//        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
//        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .3);                    // get away from the lander
        linearOp.sleep(sleepTime);
        myMechDrive.driveForward(.3, .5);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);

        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
            case LEFT:

                myMechDrive.strafeLeft(SPD_DRIVE_HIGH, 1.5);
                linearOp.sleep(sleepTime);
                myMechDrive.rotateRight(SPD_DRIVE_MED, .5);         // fixing Gyro issue
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(36, myMechDrive);           // different (34)
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArm(1000, SPD_ARM_MED, 2);
                linearOp.sleep(sleepTime);

                myIntakeSpinnerMotor.intakeSpinner(-1);
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntakeArm(0, SPD_ARM_MED,2);
                linearOp.sleep(sleepTime);

                myIntakeRotator.raisedRotater();
                linearOp.sleep(sleepTime);
                break;

            case MIDDLE:
                myMechDrive.strafeLeft(SPD_DRIVE_HIGH, .3);
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(4, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);
                myIntakeExtenderArm.extendIntakeArm(1000, SPD_ARM_MED, 2);
                linearOp.sleep(sleepTime);


                myIntakeSpinnerMotor.intakeSpinner(-1);
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntakeArm(0, SPD_ARM_MED,2);
                linearOp.sleep(sleepTime);
                myIntakeRotator.raisedRotater();
                linearOp.sleep(sleepTime);
                break;

            case RIGHT:

                myMechDrive.strafeRight(SPD_DRIVE_HIGH, .5);
                linearOp.sleep(sleepTime);
                myMechDrive.rotateLeft(SPD_DRIVE_MED, .3);        // fixing Gyro issue
                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(-14, myMechDrive);          // Gyro angles appears correct.
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeRotator.loweredRotater();
                linearOp.sleep(sleepTime);
                myIntakeExtenderArm.extendIntakeArm(1000, SPD_ARM_MED,2);
                linearOp.sleep(sleepTime);

                myIntakeSpinnerMotor.intakeSpinner(-1);
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntakeArm(0, SPD_ARM_MED,2);
                linearOp.sleep(sleepTime);

                myIntakeRotator.raisedRotater();
                linearOp.sleep(sleepTime);
                break;
        }
    }


    public void RotateDriveTowardCrater (GyroCompetition myGyro, MecanumDrive myMechDrive) {
        //myMechDrive.driveBackward(SPD_DRIVE_MED,.1);
        myMechDrive.rotateRight(SPD_DRIVE_MED,.7 );                      // fixing Gyro issue
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(0, myMechDrive);                   // gyro towards the crater after dropping tm

        switch (goldPosition) {
            case LEFT:
                myMechDrive.driveForward(SPD_DRIVE_MED,1.9);                  // drive toward the wall near the crater

            case MIDDLE:
                myMechDrive.driveForward(SPD_DRIVE_MED,2.1);                 // drive toward the wall near the crater

            case RIGHT:
                myMechDrive.driveForward(SPD_DRIVE_MED,2.5);                 // drive toward the wall near the crater

        }
    }

    public void knockingOffSecondMineral (GyroCompetition myGyro, MecanumDrive myMechDrive) {
        myMechDrive.rotateRight(SPD_DRIVE_MED,.6);
        myMechDrive.driveForward(SPD_DRIVE_HIGH,2);
        myMechDrive.rotateLeft(SPD_DRIVE_MED,.5);
        myGyro.gyroOrientMecanum(0, myMechDrive);

        switch(goldPosition) {
            case LEFT:
                myMechDrive.rotateLeft(SPD_DRIVE_MED,.4);
                myGyro.gyroOrientMecanum(0, myMechDrive);
                myMechDrive.driveForward(SPD_DRIVE_HIGH, .8);
            case MIDDLE:
                myMechDrive.rotateLeft(SPD_DRIVE_MED,.4);
                myGyro.gyroOrientMecanum(0, myMechDrive);
                myMechDrive.driveForward(SPD_DRIVE_HIGH, .8);
            case RIGHT:
                myMechDrive.rotateRight(SPD_DRIVE_MED,.4);
                myGyro.gyroOrientMecanum(0, myMechDrive);
                myMechDrive.driveForward(SPD_DRIVE_HIGH, .8);

        }

    }



  }

