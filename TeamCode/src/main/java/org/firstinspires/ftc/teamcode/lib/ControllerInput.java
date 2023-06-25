package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Simple class that registers button presses.
 * Optimized for extra speed.
 */

public class ControllerInput {
    private final Gamepad gamepad;
    private int dpad_up, dpad_down, dpad_left, dpad_right;
    private int x, y, a, b;
    private int square, circle, triangle, cross, share, options;
    private int left_bumper, right_bumper;

    public double left_stick_x, right_stick_x, left_stick_y, right_stick_y;
    public double left_trigger, right_trigger;

    public ControllerInput(Gamepad g) {
        gamepad = g;
    }

    void updateButtons() {
        if (gamepad.x) {
            ++x;
        } else {
            x = 0;
        }
        if (gamepad.y) {
            ++y;
        } else {
            y = 0;
        }
        if (gamepad.a) {
            ++a;
        } else {
            a = 0;
        }
        if (gamepad.b) {
            ++b;
        } else {
            b = 0;
        }
        if (gamepad.square) {
            ++square;
        } else {
            square = 0;
        }
        
        if (gamepad.circle) {
            ++circle;
        } else {
            circle = 0;
        }
        
        if (gamepad.cross) {
            ++cross;
        } else {
            cross = 0;
        }
        if (gamepad.triangle) {
            ++triangle;
        } else {
            triangle = 0;
        }
        if (gamepad.share) {
            ++share;
        } else {
            share = 0;
        }
        if (gamepad.options) {
            ++options;
        } else {
            options = 0;
        }
    }

    void updateArrows() {
        if (gamepad.dpad_up) {
            ++dpad_up;
            return;
        } else {
            dpad_up = 0;
        }
        if (gamepad.dpad_down) {
            ++dpad_down;
            return;
        } else {
            dpad_down = 0;
        }
        if (gamepad.dpad_left) {
            ++dpad_left;
            return;
        } else {
            dpad_left = 0;
        }
        if (gamepad.dpad_right) {
            ++dpad_right;
        } else {
            dpad_right = 0;
        }
    }

    void updateBumpers() {
        if (gamepad.left_bumper) {
            ++left_bumper;
        } else {
            left_bumper = 0;
        }
        if (gamepad.right_bumper) {
            ++right_bumper;
        } else {
            right_bumper = 0;
        }
    }

    void updateSticks() {
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
    }

    void updateTriggers() {
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
    }

    public void update() {
        updateButtons();
        updateBumpers();
        updateArrows();
        updateTriggers();
        updateSticks();
    }

    public boolean dpadUp() {
        return 0 < dpad_up;
    }

    public boolean dpadDown() {
        return 0 < dpad_down;
    }

    public boolean dpadLeft() {
        return 0 < dpad_left;
    }

    public boolean dpadRight() {
        return 0 < dpad_right;
    }

    public boolean X() {
        return 0 < x;
    }

    public boolean Y() {
        return 0 < y;
    }

    public boolean A() {
        return 0 < a;
    }

    public boolean B() {
        return 0 < b;
    }

    public boolean circle() {
        return 0 < circle;
    }

    public boolean square() {
        return 0 < square;
    }

    public boolean cross() {
        return 0 < cross;
    }

    public boolean triangle() {
        return 0 < triangle;
    }
    public boolean share() {
        return 0 < share;
    }
    public boolean options() {
        return 0 < options;
    }
    public boolean leftBumper() {
        return 0 < left_bumper;
    }

    public boolean rightBumper() {
        return 0 < right_bumper;
    }

    public boolean dpadUpOnce() {
        return 1 == dpad_up;
    }

    public boolean dpadDownOnce() {
        return 1 == dpad_down;
    }

    public boolean dpadLeftOnce() {
        return 1 == dpad_left;
    }

    public boolean dpadRightOnce() {
        return 1 == dpad_right;
    }

    public boolean XOnce() {
        return 1 == x;
    }

    public boolean YOnce() {
        return 1 == y;
    }

    public boolean AOnce() {
        return 1 == a;
    }

    public boolean BOnce() {
        return 1 == b;
    }
    public boolean circleOnce() {
        return 1 == circle;
    }

    public boolean squareOnce() {
        return 1 == square;
    }

    public boolean crossOnce() {
        return 1 == cross;
    }

    public boolean triangleOnce() {
        return 1 == triangle;
    }

    public boolean shareOnce() {
        return 1 == share;
    }
    public boolean optionsOnce() {
        return 1 == options;
    }
    public boolean leftBumperOnce() {
        return 1 == left_bumper;
    }

    public boolean rightBumperOnce() {
        return 1 == right_bumper;
    }
}
