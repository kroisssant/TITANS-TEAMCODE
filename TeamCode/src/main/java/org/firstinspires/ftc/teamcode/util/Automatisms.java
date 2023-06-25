package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeVechi;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;

import org.firstinspires.ftc.teamcode.mechanisms.Lifter.LIFTER_LEVEL;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeVechi.INTAKE_STATE;

public class Automatisms {
    public IntakeVechi intakeVechi;
    public Lifter lifter;

    public Automatisms(Lifter _lifter, IntakeVechi _intakeVechi) {
        this.lifter = _lifter;
        this.intakeVechi = _intakeVechi;
    }

    public void goToState(LIFTER_LEVEL currentLifterState, INTAKE_STATE currentIntakeState, LIFTER_LEVEL newLifterState,
                          INTAKE_STATE newIntakeState) {
        if (newLifterState == Lifter.LIFTER_LEVEL.DOWN && newIntakeState == INTAKE_STATE.INSIDE) {
            // TO: DOWN IN

            if (currentLifterState == LIFTER_LEVEL.HIGH && currentIntakeState == INTAKE_STATE.OUTSIDE) {
                // FROM: HIGH OUT
                lifter.setTargetTicks(0, LIFTER_LEVEL.DOWN.ticks);
                intakeVechi.swingSetTargetTicks(0, INTAKE_STATE.INSIDE.ticks);
                return;
            }

            if (currentLifterState == LIFTER_LEVEL.HIGH && currentIntakeState == INTAKE_STATE.INSIDE) {
                // FROM: HIGH IN
                lifter.setTargetTicks(0, LIFTER_LEVEL.DOWN.ticks);
                return;
            }

            if (currentLifterState == LIFTER_LEVEL.MID && currentIntakeState == INTAKE_STATE.OUTSIDE) {
                // FROM: MID OUT
                lifter.setTargetTicks(0, newLifterState.ticks);
                intakeVechi.swingSetTargetTicks(200, newIntakeState.ticks);
                return;
            }
        }

        if (newLifterState == LIFTER_LEVEL.HIGH && newIntakeState == INTAKE_STATE.OUTSIDE) {
            // TO: HIGH OUT

            if (currentLifterState == LIFTER_LEVEL.DOWN && (currentIntakeState == INTAKE_STATE.INSIDE || currentIntakeState == INTAKE_STATE.INIT)) {
                // FROM: DOWN IN (or INIT)
                intakeVechi.swingSetTargetTicks(500, INTAKE_STATE.OUTSIDE.ticks);
                lifter.setTargetTicks(0, LIFTER_LEVEL.HIGH.ticks);
                return;
            }
        }

        if (newLifterState == LIFTER_LEVEL.HIGH && newIntakeState == INTAKE_STATE.INSIDE) {
            // TO: HIGH IN

            if (currentLifterState == LIFTER_LEVEL.DOWN && (currentIntakeState == INTAKE_STATE.INSIDE || currentIntakeState == INTAKE_STATE.INIT)) {
                // FROM: DOWN IN (or INIT)
                intakeVechi.swingSetTargetTicks(300, INTAKE_STATE.INSIDE.ticks);
                lifter.setTargetTicks(50, LIFTER_LEVEL.HIGH.ticks);
                return;
            }
        }

        if (newLifterState == LIFTER_LEVEL.MID && newIntakeState == INTAKE_STATE.OUTSIDE) {
            // TO: MID OUT

            if (currentLifterState == LIFTER_LEVEL.DOWN && (currentIntakeState == INTAKE_STATE.INSIDE || currentIntakeState == INTAKE_STATE.INIT)) {
                // FROM: DOWN IN (or INIT)
                intakeVechi.swingSetTargetTicks(300, INTAKE_STATE.OUTSIDE.ticks);
                lifter.setTargetTicks(0, LIFTER_LEVEL.MID.ticks);
                return;
            }
        }

        if (newLifterState == LIFTER_LEVEL.MID && newIntakeState == INTAKE_STATE.INSIDE) {
            // TO: MID IN

            if (currentLifterState == LIFTER_LEVEL.HIGH && currentIntakeState == INTAKE_STATE.OUTSIDE) {
                // FROM: HIGH OUT
                intakeVechi.swingSetTargetTicks(0, INTAKE_STATE.INSIDE.ticks);
                lifter.setTargetTicks(0, LIFTER_LEVEL.MID.ticks);
                return;
            }
        }

        if (newLifterState == LIFTER_LEVEL.LOW && newIntakeState == INTAKE_STATE.INSIDE) {
            // TO: LOW IN

            if (currentLifterState == LIFTER_LEVEL.HIGH && currentIntakeState == INTAKE_STATE.OUTSIDE) {
                // FROM: HIGH OUT
                intakeVechi.swingSetTargetTicks(0, INTAKE_STATE.INSIDE.ticks);
                lifter.setTargetTicks(0, LIFTER_LEVEL.LOW.ticks);
                return;
            }
        }

        if (newLifterState == LIFTER_LEVEL.DOWN && newIntakeState == INTAKE_STATE.OUTSIDE){
            // TO: DOWN OUT

            if(currentLifterState == LIFTER_LEVEL.DOWN && (currentIntakeState == INTAKE_STATE.INSIDE || currentIntakeState == INTAKE_STATE.INIT)) {
                // FROM: DOWN IN (OR INIT)
                lifter.setTargetTicks(0, LIFTER_LEVEL.MID.ticks);
                intakeVechi.swingSetTargetTicks(500, INTAKE_STATE.OUTSIDE.ticks);
                lifter.setTargetTicks(800, LIFTER_LEVEL.DOWN.ticks);
                return;
            }
        }

        // no specific automation, just go there (manual handling)
        intakeVechi.swingSetTargetTicks(0, newIntakeState.ticks);
        lifter.setTargetTicks(0, newLifterState.ticks);
    }
}

