DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button

// mechanical button
//DebounceIn mechanical_button(PC_6); // create DigitalIn object to evaluate mechanical button, you
                                   // need to specify the mode for proper usage, see below
//mechanical_button.fall(&mechanical_button_pressed_fcn);    // sets pullup between pin and 3.3 V, so that there
                                   // is a defined potential


void mechanical_button_pressed_fcn()
{
    button_pressed++;
    printf("increase button pressed %d", button_pressed);
    return;
}

