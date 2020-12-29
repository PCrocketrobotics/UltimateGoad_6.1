package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    //public BNO055IMU imu_hub1;
    MecanumDriveTrain driveTrain;
    ComputerVision    computerVision;
    WobbleGripper     wobbleGripper;
    RingControl       ringControl;
    IMUControl        imuControl;

    ElapsedTime autonomusTime;
    final LinearOpMode opMode;

    public Robot (LinearOpMode opMode){
        this.opMode         = opMode;
        this.autonomusTime  = new ElapsedTime();
        this.driveTrain     = new MecanumDriveTrain(this);
        this.computerVision = new ComputerVision(this);
        this.wobbleGripper  = new WobbleGripper(this);
        this.ringControl    = new RingControl(this);
        this.imuControl     = new IMUControl(this);

    }

    public MecanumDriveTrain getDriveTrain()     {return this.driveTrain;}
    public ComputerVision    getComputerVision() {return this.computerVision;}
    public WobbleGripper     getWobbleGripper()  {return this.wobbleGripper;}
    public RingControl       getRingControl()    {return this.ringControl;}
    public IMUControl        getImuControl()     {return this.imuControl;}
}
