package org.firstinspires.ftc.teamcode.robot.old.classes.sub;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by johnduval on 10/9/17.
 */

public class glyphArms {
    //keep to reference in Notebook - was value to get arm open position outside frame.
    //with single bracket - open past frame
    //single bracket closed
    //double leftClosePosition = 0.8;
    //with single bracket - parallel open
    //double leftOpenPosition = 0.28;
    //with single bracket - open past frame
    //double leftOpenPosition = 0.43;
    // single bracket closed
    //double rightClosePosition = 0.34;
    //keep to reference in Notebook - was value to get arm open position outside frame.
    //with single bracket - parallel open
    //double rightOpenPosition = 0.86;
    //with single bracket - open past frame
    //double rightOpenPosition = 0.705;

    //with channel
    double topLeftClosePosition =.355; //closed to grab glyphs
    double topLeftSlightlyOpenPosition = .31;  //sightly open for exiting CryptoBox and picking up Glyphs
    double topLeftOpenLoweredPosition = .24; //fully open when glyph lift is lowered - to the frame
    double topLeftOpenPosition = .03; //fully open when raised
    double topLeftRelicPosition = 0;

    double bottomLeftClosePosition =.345;
    double bottomLeftSlightlyOpenPosition = .39;
    double bottomLeftOpenLoweredPosition = .45;
    double bottomLeftOpenPosition = .63;

    double topRightClosePosition = .495;
    double topRightSlightlyOpenPosition = .56;
    double topRightOpenLoweredPosition = .63;
    double topRightOpenPosition = .78;

    double bottomRightClosePosition = .695;
    double bottomRightSlightlyOpenPosition = .65;
    double bottomRightOpenLoweredPosition = .58;
    double bottomRightOpenPosition = .41;

    public Servo topLeftGlyphArm, bottomLeftGlyphArm, topRightGlyphArm, bottomRightGlyphArm;
    public glyphArms(Servo tLGA, Servo bLGA, Servo tRGA, Servo bRGA) {
        topLeftGlyphArm = tLGA;
        bottomLeftGlyphArm = bLGA;
        topRightGlyphArm = tRGA;
        bottomRightGlyphArm = bRGA;
    }

    public void closeGlyphArms () {
        topLeftGlyphArm.setPosition(topLeftClosePosition);
        bottomLeftGlyphArm.setPosition(bottomLeftClosePosition);
        topRightGlyphArm.setPosition(topRightClosePosition);
        bottomRightGlyphArm.setPosition(bottomRightClosePosition);
    }

    public void slightlyOpenGlyphArms () {
        topLeftGlyphArm.setPosition(topLeftSlightlyOpenPosition);
        bottomLeftGlyphArm.setPosition(bottomLeftSlightlyOpenPosition);
        topRightGlyphArm.setPosition(topRightSlightlyOpenPosition);
        bottomRightGlyphArm.setPosition(bottomRightSlightlyOpenPosition);
    }
    public void openLoweredGlyphArms () {
        topLeftGlyphArm.setPosition(topLeftOpenLoweredPosition);
        bottomLeftGlyphArm.setPosition(bottomLeftOpenLoweredPosition);
        topRightGlyphArm.setPosition(topRightOpenLoweredPosition);
        bottomRightGlyphArm.setPosition(bottomRightOpenLoweredPosition);
    }

    public void openRaisedGlyphArms () {
        topLeftGlyphArm.setPosition(topLeftOpenPosition);
        bottomLeftGlyphArm.setPosition(bottomLeftOpenPosition);
        topRightGlyphArm.setPosition(topRightOpenPosition);
        bottomRightGlyphArm.setPosition(bottomRightOpenPosition);
    }

    public void relicGlyphArms () {
        topLeftGlyphArm.setPosition(topLeftRelicPosition);
        bottomLeftGlyphArm.setPosition(bottomLeftOpenLoweredPosition);
        topRightGlyphArm.setPosition(topRightOpenLoweredPosition);
        bottomRightGlyphArm.setPosition(bottomRightOpenLoweredPosition);
    }
}
