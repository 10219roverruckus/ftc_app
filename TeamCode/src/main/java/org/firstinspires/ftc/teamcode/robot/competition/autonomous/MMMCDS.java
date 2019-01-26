package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;


public class MMMCDS {

    public MMMCDS() {

    }
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




    // new methods for this class


    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo)  {

        linearOp.telemetry.addData("MINERAL", goldPosition);
        linearOp.telemetry.update();
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .3);                    // get away from the lander
        linearOp.sleep(sleepTime);
        myMechDrive.driveForward(SPD_DRIVE_HIGH, .3);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH,.4);

        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
            case LEFT:
                myMechDrive.rotateLeft(SPD_DRIVE_MED, .4);          // fixing Gyro issue
                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(36, myMechDrive);           // different (34)
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeRotator.mineralRotateLowerEncoder();                // lower rotater
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArmAuto();                  // extend extender arm
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntakeArmAuto();                 // retract arm after hitting off mineral
                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateRaiseEncoder();                // raise rotater
                linearOp.sleep(sleepTime);

                break;

            case MIDDLE:
                myMechDrive.rotateLeft(SPD_DRIVE_MED,.1);
                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(4, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeRotator.mineralRotateLowerEncoder();                // lower rotater
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArmAuto();                  // extend extender arm
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntakeArmAuto();                 // retract arm after hitting off mineral
                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateRaiseEncoder();                // raise rotater
                linearOp.sleep(sleepTime);

                break;

            case RIGHT:
                myMechDrive.rotateRight(SPD_DRIVE_MED, .3);         // fixing Gyro issue
                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(-14, myMechDrive);          // Gyro angles appears correct.
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeRotator.mineralRotateLowerEncoder();                // lower rotater
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArmAuto();                  // extend extender arm
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntakeArmAuto();                 // retract arm after hitting off mineral
                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateRaiseEncoder();                // raise rotater
                linearOp.sleep(sleepTime);

                break;
        }
    }







    public void RotateDriveWall (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {

        myMechDrive.rotateLeft(SPD_DRIVE_MED, .7);              // fixing Gyro issue of over rotating
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(74, myMechDrive);                //orients self with red tape so parallel to tape.
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);
        myMechDrive.driveForward(SPD_DRIVE_HIGH, 3.7);
        linearOp.sleep(sleepTime);
    }




    public void RotateDriveTowardDepot(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDistance) {
        myMechDrive.rotateLeft(SPD_DRIVE_MED, .4);              // fixing Gyro issue
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(137, myMechDrive);              // Orient for straight drive to depot
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);

        myMechDrive.setMotorPowerStrafeRight(.3);                      // Align to wall
        linearOp.sleep(500);                               // Time for straffing
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);

        Color.RGBToHSV((int) (myRevColorDistance.revColorSensor.red() * SCALE_FACTOR),     // Move backwards until color detected
                (int) (myRevColorDistance.revColorSensor.green() * SCALE_FACTOR),
                (int) (myRevColorDistance.revColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        while (hsvValues[0] > RED_THRESHOLD && hsvValues[0] < BLUE_THRESHOLD) {
            Color.RGBToHSV((int) (myRevColorDistance.revColorSensor.red() * SCALE_FACTOR),
                    (int) (myRevColorDistance.revColorSensor.green() * SCALE_FACTOR),
                    (int) (myRevColorDistance.revColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            myMechDrive.setMotorSpeeds(-SPD_DRIVE_MED);
            linearOp.idle();
        }
        linearOp.sleep(sleepTime);
    }





    public void LowerReleaseTM (IntakeRotator myIntakeRotator, IntakeServo myIntakeServo) {
        // extend arm and lower rotator
        // rotator will spin to release TM
        // extender will retract and rotator will raise

        myIntakeRotator.mineralRotateLowerEncoder();                // lower rotater
        linearOp.sleep(sleepTime);

        myIntakeServo.IntakeServoReverse();                         // spin to spit out the team marker
        linearOp.sleep(sleepTime);

        myIntakeRotator.mineralRotateRaiseEncoder();                //  raise rotater
        linearOp.sleep(sleepTime);

    }

    public void knockingOffSecondMineral (GyroCompetition myGyro, MecanumDrive myMechDrive, IntakeExtenderArm myIntakeExtenderArm, IntakeRotator myIntakeRotator) {
        myMechDrive.rotateLeft(SPD_DRIVE_MED, .5);
        myGyro.gyroOrientMecanum(0, myMechDrive);
        myMechDrive.driveForward(SPD_DRIVE_MED, .4);
        myMechDrive.rotateLeft(SPD_DRIVE_MED, .5);
        myGyro.gyroOrientMecanum(0, myMechDrive);

        switch (goldPosition) {
            case LEFT:
                myGyro.gyroOrientMecanum(0, myMechDrive);                   // rotate toward mineral
                myIntakeRotator.LowerIntakeRotater();                              // lower and extend arm to hit mineral
                myIntakeExtenderArm.extendIntakeArmAuto();
                myIntakeExtenderArm.retractIntakeArmAuto();
                myIntakeRotator.mineralRotateRaiseEncoder();
            case MIDDLE:
                myGyro.gyroOrientMecanum(0, myMechDrive);                   // rotate toward mineral
                myIntakeRotator.LowerIntakeRotater();                              // lower and extend arm to hit mineral
                myIntakeExtenderArm.extendIntakeArmAuto();
                myIntakeExtenderArm.retractIntakeArmAuto();
                myIntakeRotator.mineralRotateRaiseEncoder();
            case RIGHT:
                myGyro.gyroOrientMecanum(0, myMechDrive);                    // rotate toward mineral
                myIntakeRotator.LowerIntakeRotater();                               // lower and extend arm to hit mineral
                myIntakeExtenderArm.extendIntakeArmAuto();
                myIntakeExtenderArm.retractIntakeArmAuto();
                myIntakeRotator.mineralRotateRaiseEncoder();
        }
    }




    public void DriveParkInCrater (GyroCompetition myGyro, MecanumDrive myMechDrive) {

        myMechDrive.rotateRight(SPD_DRIVE_MED,.5);                          // rotate away from minerals toward wall
        myGyro.gyroOrientMecanum(0, myMechDrive);

        myMechDrive.driveForward(SPD_DRIVE_HIGH, .7);                        // drive toward wall

        myMechDrive.rotateLeft(SPD_DRIVE_MED, .5);                              // rotate to align robot with wall
        myGyro.gyroOrientMecanum(0, myMechDrive);

        myMechDrive.strafeRight(SPD_DRIVE_HIGH, 1);                          // strafe into wall

        myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.5);                       // drive and parking in crater
    }



}

