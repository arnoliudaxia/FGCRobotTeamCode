package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

public class Servoss extends Thread
{
    private double ServoPosition;
    private Servo myServo;

    public Servoss(double InputPosition,Servo InputServo)
    {
        this.ServoPosition=InputPosition;
        this.myServo=InputServo;
    }


    @Override
    public void run()
    {
        for(int i=0;i<=ServoPosition;i+=0.05)
        {
            myServo.setPosition(i);
            try
            {
                Thread.sleep(60);
            } catch(InterruptedException e)
            {
                e.printStackTrace();
            }
        }
    }
}
