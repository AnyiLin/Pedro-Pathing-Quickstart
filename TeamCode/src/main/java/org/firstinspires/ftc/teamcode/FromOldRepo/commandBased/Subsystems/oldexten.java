package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class oldexten extends SubsystemBase {
    private DcMotorEx extensionMotor;
    private double maxExtensionLimit = 1000;
    private double minExtensionLimit = 0;
    private double extensionOutPosition = 800;
    private double extensionPower = 0.75;

    public oldexten(DcMotorEx extensionMotor) {
        this.extensionMotor = extensionMotor;
        this.extensionMotor.setTargetPosition(0);
        this.extensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setExtensionOut(){
        extensionMotor.setTargetPosition(800);
    }
    public void runExtension(){
        if(extensionMotor.isBusy()){
            extensionMotor.setPower(extensionPower);
        }else{
            extensionMotor.setPower(0);
        }
    }
    public boolean isExtensionBusy(){
        return !
                extensionMotor.isBusy();
    }



}
