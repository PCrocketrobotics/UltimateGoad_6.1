package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGripper {
    static final double MAX_POS     = 1.0;
    static final double MIN_POS     = 0.0;
    Servo     wobbleservo;
    DcMotorEx wobblearm;
    Robot robot;

    public WobbleGripper(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        //Wheel Drive Motors Setup
        wobbleservo = robot.opMode.hardwareMap.get(Servo.class, "wobbleservo");
        wobblearm =   robot.opMode.hardwareMap.get(DcMotorEx.class, "wobblearm");
        wobbleservo.setPosition(0);
    }

    public  void  setArmPower(double wobblepower){
        wobblearm.setPower(wobblepower);
    };

    public void setWobbleGripper(double wobbleposition){

        wobbleservo.setPosition(wobbleposition);
        double currentwobbleposition = wobbleservo.getPosition();
        while (currentwobbleposition != wobbleposition){
            //wait until in position
        }

    }
}
