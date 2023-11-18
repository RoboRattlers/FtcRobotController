package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

// TODO: measure encoder ticks in full length
public class ViperSlide336mm extends LinearSlide {


    // all lengths are in inches
    public ViperSlide336mm(DcMotorEx myMotor) {
        motor = myMotor;
        fullLength = 38;
        cappedLength = 20;
        encoderTicksPerRevolution = 537.7;
        encoderTicksInFullLength = -3150;
    }

}
