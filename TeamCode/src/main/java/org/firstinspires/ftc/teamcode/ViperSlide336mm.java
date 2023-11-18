package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ViperSlide336mm extends LinearSlide {


    // all lengths are in inches
    public ViperSlide336mm(DcMotorEx myMotor) {
        motor = myMotor;
        fullLength = 38;
        cappedLength = 30;
        encoderTicksPerRevolution = 537.7;
        encoderTicksInFullLength = -3150;
    }

}
