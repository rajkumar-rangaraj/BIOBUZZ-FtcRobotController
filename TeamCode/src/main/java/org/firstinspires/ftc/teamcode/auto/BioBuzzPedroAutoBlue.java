package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BioBuzz AUTO BLUE", group = "BioBuzz", preselectTeleOp = "BioBuzz TeleOp")
public class BioBuzzPedroAutoBlue extends BioBuzzPedroAutoBase {
    @Override
    protected AutoField.Alliance alliance() {
        return AutoField.Alliance.BLUE;
    }
}