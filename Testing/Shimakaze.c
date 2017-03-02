#ifndef SHIMAKAZE_C
#define SHIMAKAZE_C

#include "../Enterprise.c"

/* Clawbot control state. */
struct control_t {
    signed char left;
    signed char right;

    bool up;
    bool down;

    bool open;
    bool close;
};

void controlToMotors(const control_t state) {
    motor[left] = state->left;
    motor[right] = state->right;

    if(state->open) {
        motor[claw] = 127;
    } else if(state->close) {
        motor[claw] = -127;
    } else {
        motor[claw] = 0;
    }

    if(state->up) {
        motor[arm] = 127;
    } else if(state->down) {
        motor[arm] = -127;
    } else {
        motor[arm] = 0;
    }
}

void joystickToControl(control_t* state) {
    state->left = vexRT[Ch3];
    state->right = vexRT[Ch2];

    state->up = vexRT[Btn5U];
    state->down = vexRT[Btn5D];

    state->open = vexRT[Btn6U];
    state->close = vexRT[Btn6D];
}

void replayToControl(control_t* state, replay_t* replay) {
    state->left = (signed char)readNextByte(replay);
    state->right = (signed char)readNextByte(replay);

    unsigned char btnState = readNextByte(replay);
    state->up = TEST_BIT(btnState, 0);
    state->down = TEST_BIT(btnState, 1);
    state->open = TEST_BIT(btnState, 2);
    state->close = TEST_BIT(btnState, 3);
}

void controlToReplay(const control_t state, replay_t* replay) {
    writeByte(replay, (unsigned char)state.left);
    writeByte(replay, (unsigned char)state.right);

    unsigned char btnState = 0;
    btnState |= (state.up ? 1 : 0) << 0;
    btnState |= (state.down ? 1 : 0) << 1;
    btnState |= (state.open ? 1 : 0) << 2;
    btnState |= (state.close ? 1 : 0) << 3;

    writeByte(replay, btnState);
}

#endif /* end of include guard: SHIMAKAZE_C */
