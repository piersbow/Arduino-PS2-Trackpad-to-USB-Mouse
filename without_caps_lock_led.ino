#include "HID-Project.h"    // tested with 2.6.1
#include <ArduinoQueue.h>   // tested with 1.2.3


#define CAPS_LOCK_LED 9
#define PS2_DAT 11
#define PS2_CLK 10
#define CLICK_BUTTON 2      // should be analog pin

#define IGNORE_PACKETS_START 4  // the first packets to be ignored to prevent incorrect readings
#define IGNORE_PACKETS_END 4    // the last packets to be ignored to prevent incorrect readings

#define CHECK_CAPS_LOCK_FREQUENCY 10


// mouse behaviour
#define Y_SCALE 0.78
#define TAP_MAX_LENGTH 4
#define DOUBLE_TAP_MAX_TIME 20
#define THREE_FINGER_TAP_FOR_MIDDLE_BUTTON 1


// used for acceleration profile, mouse vel = SPEED*dist + ACCEL*(dist^2), where dist is distance finger moved
#define SPEED 0.18
#define ACCEL 0.0001


// used for determining if clicks/taps should be right clicks
#define RIGHT_CLICK_MIN_X 3800
#define RIGHT_CLICK_MAX_Y_TAP 2000
#define RIGHT_CLICK_MAX_Y_CLICK 2500


// used for scrolling behaviour
#define SCROLL_SPEED -0.018          // negative to reverse direction
#define SCROLL_MOMENTUM 0.7         // must be < 1, larger means more momentum
#define MIN_SCROLL_MOMENTUM 0.005


// w is num of fingers on touchpad, 4 = 1, 0 = 2, 1 = >2, anything else ignore
// z is finger pressure, i mainly use it to tell if a finger is on the pad
// x and y are the coordinates of finger
int w, z, x, y, click;  // used for trackpad output

int packets_since_tap = 0; // used for double tap to drag, when there is a tap it is set to DOUBLE_TAP_MAX_TIME and
// then counts down till 0, if there is a tap before 0, this is treated as a drag

float scroll_vel = 0;

int total_scroll = 0;
float ideal_scroll = 0;
// the scrolling function only accepts integers, but sometimes a non integer amount of scroll is specified.
// this is used to see the different between actual and ideal scrolling, so that it can be compensated for.

bool prev_click = 0;
// used for the physical click so we know if we need to release the button.

// initial setup
void setup() {
    pinMode(CLICK_BUTTON, INPUT);
    pinMode (CAPS_LOCK_LED, OUTPUT);
    //Serial.begin(9600);
    //while(!Serial);

    // begin touchpad
    enable_touchpad();

    // begin virtual mouse

    Mouse.begin();
    // begin virtual keyboard for caps lock led
    //BootKeyboard.begin();
}



// loop
void loop() {
    // wait until finger is on trackpad
    do {
        //read_trackpad_with_caps_lock();
        read_trackpad();
        // whilst doing this, apply any scrolling momentum
        if(abs(scroll_vel) > MIN_SCROLL_MOMENTUM){
            ideal_scroll += scroll_vel;

            Mouse.move(0, 0, ideal_scroll - total_scroll);

            total_scroll = ideal_scroll;

            scroll_vel *= SCROLL_MOMENTUM;
        }

        if(packets_since_tap){  // if packets_since_tap is non zero, subtract one from it
            packets_since_tap--;
        }
    }
    while(!z);

    total_scroll = 0;
    ideal_scroll = 0;

    // if finger on pad and we have a previous measurement for position
    // once finger on pad stop any scrolling momentum
    scroll_vel = 0;

    ArduinoQueue<int> w_queue(IGNORE_PACKETS_END + 1);  // plus one becasue i was having problems with tap actions
    w_queue.enqueue(4); // 4 is standard single finger
    //ArduinoQueue<int> z_queue(IGNORE_PACKETS_END);    // i dont think a queue for z is usefull, as it is only used
    // for telling if a finger is present
    ArduinoQueue<int> x_queue(IGNORE_PACKETS_END);
    ArduinoQueue<int> y_queue(IGNORE_PACKETS_END);
    //ArduinoQueue<int> click_queue(IGNORE_PACKETS_END);    // clicks are not handled by the trackpad, but by a
    // physical button, so there is no need to queue them

    // ignore first readings to remove initial errors
    for(int i = 0; i<IGNORE_PACKETS_START; i++){
        read_trackpad();
        check_click();
    }

    // add delay of packets so we can ignore the last few packets
    for(int i = 0; i<IGNORE_PACKETS_END; i++){
        read_trackpad();
        w_queue.enqueue(w);
        x_queue.enqueue(x);
        y_queue.enqueue(y);
        check_click();
    }

    int prev_x, prev_y, total_x = 0, total_y = 0, count = 0, scrolling = 0;
    // total x/y used for taps, if a tap was triggered and the mouse moved during this, move it back to the initial
    // position before clicking

    bool drag = 0;
    if(packets_since_tap && w == 4){    // if single finger and soon after tap, hold down mouse
        Mouse.press();
        drag = 1;
    }

    while(z) {
        read_trackpad();
        check_click();
        //if(!(count%CHECK_CAPS_LOCK_FREQUENCY)){
        //    check_caps_lock();
        //}

        if(z){
            // if finger is still on touchpad, then a movement is being done
            prev_x = x_queue.dequeue();
            prev_y = y_queue.dequeue();
            w_queue.dequeue();

            w_queue.enqueue(w);
            x_queue.enqueue(x);
            y_queue.enqueue(y);

            count++;
        }

        float d_x = (x_queue.getHead() - prev_x);
        float d_y = (prev_y - y_queue.getHead()) * Y_SCALE;  // y is inverted and scaled

        int w_val = w_queue.getHead();
        // lock into scrolling once started
        if(scrolling){
            w_val = 0;
        }



        if(w_val == 4 || drag){
            //Serial.println("move");
            // calc new distance ratio with acceleration
            float dist_accl = SPEED + (ACCEL * sqrt(pow(d_x, 2) + pow(d_y, 2)));

            // apply this
            d_x *= dist_accl;
            d_y *= dist_accl;

            // move mouse
            total_x += d_x;
            total_y += d_y;
            Mouse.move(d_x, d_y);
        }else if(w_val == 0){
            //Serial.println("scroll");
            // once 2 fingers are detected, you become locked into scrolling mode
            scrolling = 1;

            scroll_vel = d_y * SCROLL_SPEED;
            ideal_scroll += scroll_vel;

            Mouse.move(0, 0, ideal_scroll - total_scroll);

            total_scroll = ideal_scroll;
        }
    }

    if(count < TAP_MAX_LENGTH){
        // if only a few readings were taken, tap
        // return mouse to initial position
        Mouse.move(-total_x, -total_y, -total_scroll);
        scroll_vel = 0;
        //Serial.println(w_queue.itemCount());
        int w_val = w_queue.dequeue();

        // if w_val is not consistent, default to 4 (single finger)
        if(w_val != w_queue.getHead()){
            w_val = 4;
        }

        if(w_val == 4){
            // if a single finger tap it may still be a right click if it is at the bottom right of trackpad
            if(x_queue.getHead() > RIGHT_CLICK_MIN_X && y_queue.getHead() < RIGHT_CLICK_MAX_Y_TAP){
                // right click
                Mouse.click(MOUSE_RIGHT);
            }else{
                Mouse.click();
                packets_since_tap = DOUBLE_TAP_MAX_TIME;
            }
        } else if(w_val == 0){
            Mouse.click(MOUSE_RIGHT);
        } else if(w_val == 1 && THREE_FINGER_TAP_FOR_MIDDLE_BUTTON){
            Mouse.click(MOUSE_MIDDLE);
        }
    }
    // at end release all mouse buttons
    Mouse.releaseAll();
}

void check_click(){
    if(click && !prev_click){
        if(x > RIGHT_CLICK_MIN_X && y < RIGHT_CLICK_MAX_Y_CLICK){
            // right click
            Mouse.click(MOUSE_RIGHT);
        }
        prev_click = 1;
        Mouse.press();
    } else if(!click && prev_click){
        prev_click = 0;
        Mouse.releaseAll();
    }

}

//void check_caps_lock(){
//    if (BootKeyboard.getLeds() & LED_CAPS_LOCK){
//        digitalWrite(CAPS_LOCK_LED, HIGH);
//    }else{
//        digitalWrite(CAPS_LOCK_LED, LOW);
//    }
//}

// PS2 other commands
void send_special(uint8_t arg) {
    // sends a bytes as 4 bytes between 0x00 and 0x03
    // basically sends 4 base-4 digits, each prefaced with a 0xE8
    for (uint8_t i = 0; i < 4; i++) {
        write(0xE8);
        write((arg >> (6 - 2 * i)) & 3);
    }
}

// PS2 high level

void enable_touchpad(){
    // reset touchpad, response = ACK 0xAA 0x00
    write(0XFF);
    read();
    read();
    read();

    //get id, response = ACK 0x00
    write(0xF3);
    write(0x0A);
    write(0xF2);
    read();
    read();

    // E6 report, response = ACK 0x00 0x00 0x0A
    write(0xE8);
    write(0x00);
    write(0xE6);
    write(0xE6);
    write(0xE6);
    write(0xE9);
    read();
    read();
    read();
    read();

    // not sure what this does, but it's important, response  = ACK 0x10 0x00 0x0A
    write(0xE8);
    write(0x00);
    write(0xE7);
    write(0xE7);
    write(0xE7);
    write(0xE9);
    read();
    read();
    read();
    read();


    // not sure what this does, but it's important, response = ACK 0x00
    write(0xF3);
    write(0xC8);
    write(0xF3);
    write(0x64);
    write(0xF3);
    write(0x50);
    write(0xF2);
    read();
    read();
    // reset
    write(0xFF);

    // identify touchpad, pg 41, response = ACK 0x02 0x47 0x18
    send_special(0x0);
    write(0xE9);

    // read touchpad modes, pg 41
    send_special(0x01);
    write(0xE9);

    // read touchpad capabilities, pg 42
    send_special(0x02);
    write(0xE9);

    // read touchpad extended model id, pg 48
    send_special(0x09);
    write(0xE9);
    read();
    //Serial.print("Read extended model id: 0x");
    //print_reads(3);

    // read touchpad capabilities continued, pg 48
    send_special(0x0C);
    write(0xE9);
    read();
    //Serial.print("Read touchpad capabilities continued: 0x");
    //print_reads(3);


    // set to absolute mode with high packet rate and w mode (C1)

    // for extended w mode use c5 but i dont think that works
    //Serial.println("Read set to absolute high packet and w mode");
    send_special(0xC1);
    write(0xF3);
    write(0x14);
    read();

    // read touchpad modes, pg 41
    send_special(0x01);
    write(0xE9);
    read();
    //Serial.print("Read touchpad modes:");
    //print_reads(3);

    // enable touchpad, this makes it send data continously, evene when we dont ask for it
    write(0xF4);
    read();
}


//// will check caps lock whilst performing read
//uint8_t read_with_caps_lock(void) {
//    uint16_t d = 0;
//
//    idle();
//    delayMicroseconds(50);
//    // wait for clock line to drop
//
//    // start bit + 8 bits data + parity + stop = 11 bits
//    for (uint8_t i = 0; i < 11; i++) {
//        while (digitalRead(PS2_CLK)){
//            check_caps_lock();
//        }
//
//        if (digitalRead(PS2_DAT))
//            d |= _BV(i);
//
//        while (!digitalRead(PS2_CLK));
//    }
//
//    inhibit();
//    // drop start bit
//    d >>= 1;
//
//    return d & 0xFF;
//
//}
//void read_trackpad_with_caps_lock(){
//    // read 6 packets from trackpad
//    uint8_t packet[6];
//
//    // this is needed because when it hasnt been used for awhile the trackpad 'sleeps' and wont send packets until
//    // it is touched to 'awake', (note the trackpad is not technically in sleep mode)
//    packet[0] = read_with_caps_lock();
//
//    // only the first read will be blocked, so other can happen as usual
//    for (uint8_t x = 1; x < 6; x++) {
//        packet[x] = read();
//    }
//    // extract values from packets, see fig 3-4 on pg 23 of 'Synaptics PS/2 TouchPad Interfacing Guide' for
//    // packet format
//
//    // num of fingers on touchpad, 4 = 1, 0 = 2, 1 = >2, anything else ignore
//    w = ((packet[0] & 0x30) >> 2) | ((packet[0] & 0x4) >> 1) | ((packet[3] & 0x4) >> 2);
//
//    // preasure of finger
//    z = packet[2];
//
//    // x/y positions
//    x = ((packet[3] & 0x10) << 8) | ((packet[1] & 0xF) << 8) | packet[4];
//    y = ((packet[3] & 0x20) << 7) | ((packet[1] & 0xF0) << 4) | packet[5];
//
//    // for some reason buttons dont work for me in PS/2 protocol
//    // int button = ((packet[0] & 0x3) << 2) | (packet[3] & 0x3);
//    // get touchpad click from wire soldered to button
//    click = analogRead(CLICK_BUTTON) < 100;
//}


void read_trackpad(){
    // read 6 packets from trackpad
    uint8_t packet[6];

    for (uint8_t x = 0; x < 6; x++) {
        packet[x] = read();
    }
    // extract values from packets, see fig 3-4 on pg 23 of 'Synaptics PS/2 TouchPad Interfacing Guide' for
    // packet format

    // num of fingers on touchpad, 4 = 1, 0 = 2, 1 = >2, anything else ignore
    w = ((packet[0] & 0x30) >> 2) | ((packet[0] & 0x4) >> 1) | ((packet[3] & 0x4) >> 2);

    // preasure of finger
    z = packet[2];

    // x/y positions
    x = ((packet[3] & 0x10) << 8) | ((packet[1] & 0xF) << 8) | packet[4];
    y = ((packet[3] & 0x20) << 7) | ((packet[1] & 0xF0) << 4) | packet[5];

    // for some reason buttons dont work for me in PS/2 protocol
    // int button = ((packet[0] & 0x3) << 2) | (packet[3] & 0x3);
    // get touchpad click from wire soldered to button
    click = analogRead(CLICK_BUTTON) < 100;
}




// bellow here is not my code, it is from the adafruit trackpad libary
// its just a PS/2 implementation




// PS2 hardware level commands
void high(uint8_t p) {
    pinMode(p, INPUT);
    digitalWrite(p, HIGH);
}

void low(uint8_t p) {
    digitalWrite(p, LOW);
    pinMode(p, OUTPUT);
}

void inhibit(void) {
    high(PS2_DAT);
    low(PS2_CLK);
}

void idle(void) {
    high(PS2_CLK);
    high(PS2_DAT);
}


// PS2 read write commands
uint8_t read(void) {
    uint16_t d = 0;

    idle();
    delayMicroseconds(50);
    // wait for clock line to drop

    // start bit + 8 bits data + parity + stop = 11 bits
    for (uint8_t i = 0; i < 11; i++) {
        while (digitalRead(PS2_CLK));

        if (digitalRead(PS2_DAT))
            d |= _BV(i);

        while (!digitalRead(PS2_CLK));
    }

    inhibit();
    // drop start bit
    d >>= 1;

    return d & 0xFF;

}

void write(uint8_t x) {
    uint16_t tosend = x;
    uint8_t parity = 1;

    for (uint8_t i = 0; i < 8; i++) {
        parity ^= (tosend >> i);
    }

    if (parity & 0x1)
        tosend |= 0x100;

    idle();
    delayMicroseconds(300);
    low(PS2_CLK);
    delayMicroseconds(100);
    low(PS2_DAT);
    delayMicroseconds(10);

    // we pull the clock line up to indicate we're ready
    high(PS2_CLK);

    // wait for the device to acknowledge by pulling it down
    while (digitalRead(PS2_CLK));

    for (uint8_t i = 0; i < 9; i++) {
        if (tosend & 0x1)
            high(PS2_DAT);
        else
            low(PS2_DAT);

        // the clock lines are driven by the -DEVICE- so we wait
        while (!digitalRead(PS2_CLK));

        while (digitalRead(PS2_CLK));

        tosend >>= 1;
    }

    // send stop bit (high)
    high(PS2_DAT);
    delayMicroseconds(50);
    while (digitalRead(PS2_CLK));

    // wait for mouse to switch modes
    while (!digitalRead(PS2_CLK) || !digitalRead(PS2_DAT));

    // inhibit any more data (we will poll!)
    inhibit();
}
