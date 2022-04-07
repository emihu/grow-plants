/* Address values in the DE1-SoC board */

/* Memory */
#define SDRAM_BASE            0xC0000000
#define FPGA_ONCHIP_BASE      0xC8000000
#define FPGA_CHAR_BASE        0xC9000000
#define A9_ONCHIP_END         0xFFFFFFFF

/* Cyclone V FPGA devices */
#define LEDR_BASE             0xFF200000
#define HEX3_HEX0_BASE        0xFF200020
#define HEX5_HEX4_BASE        0xFF200030
#define SW_BASE               0xFF200040
#define KEY_BASE              0xFF200050
#define TIMER_BASE            0xFF202000
#define PIXEL_BUF_CTRL_BASE   0xFF203020
#define CHAR_BUF_CTRL_BASE    0xFF203030

/* Interrupt controller (GIC) CPU interface(s) */
#define MPCORE_GIC_CPUIF      0xFFFEC100    // PERIPH_BASE + 0x100
#define ICCICR                0x00          // offset to CPU interface control reg
#define ICCPMR                0x04          // offset to interrupt priority mask reg
#define ICCIAR                0x0C          // offset to interrupt acknowledge reg
#define ICCEOIR               0x10          // offset to end of interrupt reg

/* Interrupt controller (GIC) distributor interface(s) */
#define MPCORE_GIC_DIST       0xFFFED000    // PERIPH_BASE + 0x1000
#define ICDDCR                0x00          // offset to distributor control reg

/* Operating modes and interrupt enable bits */
#define IRQ_MODE              0b10010
#define SVC_MODE              0b10011
#define INT_ENABLE            0b01000000
#define INT_DISABLE           0b11000000

#define EDGE_TRIGGERED        0x1
#define LEVEL_SENSITIVE       0x0
#define CPU0                  0x01          // bit-mask; bit 0 represents cpu0
#define ENABLE                0x1

/* IRQ interrupt IDs */
#define INTERVAL_TIMER_IRQ    72
#define KEYS_IRQ              73

/* Load value for A9 Private Timer */
#define TIMER_LOAD            10000000       // 1/(100 MHz) x 1x10^7 = 0.1 sec

/* VGA colors */
#define WHITE 0xFFFF
#define YELLOW 0xFFE0
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define GREY 0xC618
#define PINK 0xFC18
#define ORANGE 0xFC00
#define BLACK 0x0000

#define ABS(x) (((x) > 0) ? (x) : -(x))

/* Screen size */
#define RESOLUTION_X 320
#define RESOLUTION_Y 240

/* Constants for animation */
#define BOX_LEN 2
#define NUM_BOXES 8
#define SW_MAX 64

#define FALSE 0
#define TRUE 1

/* Include directives */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>


/*
 * STRUCT DEFINITIONS
 */
typedef struct Box_Info {
    int x;                  // (x,y) positions
    int y;
    int x_dir;              // directions, either -1 or 1
    int y_dir;
    short int color;        // colour
} Box_Info;


/*
 * GLOBAL VARIABLES
 */
int gTime;

Box_Info boxes[SW_MAX];
Box_Info old_boxes[SW_MAX];

int num_boxes;
int num_curr_boxes;
int num_old_boxes;

volatile int pixel_buffer_start; // global variable


/*
 * FUNCTION DECLARATIONs
 */
/* Set up interrupts */
void set_A9_IRQ_stack(void);
void config_GIC(void);
void config_interval_timer(void);
void config_KEYs(void);
void enable_A9_interrupts(void);

/* Interrupt service routines */
void interval_timer_ISR(void);
void pushbutton_ISR(void);

/* Set up and manage pixel buffers */
void set_pixel_buffers(void);
void wait_for_vsync();

/* Draw larger structures */
void init_boxes();
void move_boxes();
void update_old_boxes();
void clear_boxes();
void draw_box_lines();

/* Draw simple shapes and lines */
void plot_pixel(int x, int y, short int line_color);
void draw_box(int x0, int y0, short int color, int length);
void plot_line(int x0, int y0, int x1, int y1, short int line_color);
void plot_ellipse(int x0, int y0, int r1, int r2, short int color);
void plot_quad_bezier_seg(int x0, int y0, int x1, int y1, int x2, int y2, short int line_color);
void plot_quad_bezier(int x0, int y0, int x1, int y1, int x2, int y2, short int line_color);
void clear_screen();

/* Helper functions */
void swap (int *first, int *second);

#define SCALE_FACTOR 80
#define SPEED_SCALE 1

typedef struct Rand_Box_Info {
    double x;               // (x,y) positions
    double y;
    double x_dir;           // directions, either -1 or 1
    double y_dir;
    short int color;        // colour
    int reverse_x;
    int reverse_y;
} Rand_Box_Info;

typedef struct D_Point {
    double x;               // (x,y) positions
    double y;
} D_Point;

Rand_Box_Info rand_box;
Rand_Box_Info old_rand_box;
D_Point initial_point;

D_Point vector_field (double x, double y, double time) {
    D_Point point;
    point.x = sin((sin(time/SCALE_FACTOR/4)*x + sin(time/SCALE_FACTOR)*y)/SCALE_FACTOR);
    point.y = -cos((cos(time/SCALE_FACTOR)*x + cos(time/SCALE_FACTOR/4)*y)/SCALE_FACTOR);
    return point;
}
void init_rand_point () {
    rand_box.x = rand() % RESOLUTION_X;
    rand_box.y = rand() % RESOLUTION_Y;
    D_Point dir = vector_field(rand_box.x, rand_box.y, gTime);
    rand_box.x_dir = dir.x;
    rand_box.y_dir = dir.y;
    rand_box.color = (short int)(rand() % 0xFFFF);
    rand_box.reverse_x = 1;
    rand_box.reverse_y = 1;

    old_rand_box.x = rand_box.x;
    old_rand_box.y = rand_box.y;
    old_rand_box.x_dir = 0;
    old_rand_box.y_dir = 0;
    old_rand_box.color = BLACK;
    old_rand_box.reverse_x = 0;
    old_rand_box.reverse_y = 0;

    initial_point.x = rand_box.x;
    initial_point.y = rand_box.y;
}
void draw_rand_point () {
    // clear box
    draw_box((int)old_rand_box.x, (int)old_rand_box.y, 
             old_rand_box.color, 5);
    plot_quad_bezier((int)initial_point.x, (int)initial_point.y, 
                     (int)initial_point.x, (int)old_rand_box.y,
                     (int)old_rand_box.x, (int)old_rand_box.y,
                     old_rand_box.color);

    // update old box
    old_rand_box.x = rand_box.x;
    old_rand_box.y = rand_box.y;
    
    // move box
    if (rand_box.x <= 0 || rand_box.x >= RESOLUTION_X)
        rand_box.reverse_x *= -1;
    if (rand_box.y <= 0 || rand_box.y >= RESOLUTION_Y)
        rand_box.reverse_y *= -1; 
    rand_box.x += SPEED_SCALE * rand_box.x_dir * rand_box.reverse_x;
    rand_box.y += SPEED_SCALE * rand_box.y_dir * rand_box.reverse_y;

    if (rand_box.x < 0) rand_box.x = 0;
    if (rand_box.y < 0) rand_box.y = 0;

    // change direction
    D_Point dir = vector_field(rand_box.x, rand_box.y, gTime);
    rand_box.x_dir = dir.x;
    rand_box.y_dir = dir.y;

    draw_box((int)rand_box.x, (int)rand_box.y, rand_box.color, 5);
    plot_quad_bezier((int)initial_point.x, (int)initial_point.y, 
                     (int)initial_point.x, (int)rand_box.y,
                     (int)rand_box.x, (int)rand_box.y,
                     rand_box.color);
}

/*
 * FUNCTION DEFINITIONS
 */
int main(void)
{
    volatile int * pixel_ctrl_ptr = (int *)PIXEL_BUF_CTRL_BASE;
    volatile int * SW_ptr = (int *)SW_BASE;

    // initialize number of boxes
    num_curr_boxes = 0;
    num_boxes = *SW_ptr;
    if (num_boxes > SW_MAX)
        num_boxes = SW_MAX;

    // initialize location and direction of rectangles
    init_boxes();

    set_A9_IRQ_stack();      // initialize the stack pointer for IRQ mode
    config_GIC();            // configure the general interrupt controller
    config_interval_timer(); // configure interval timer to generate interrupts
    config_KEYs();           // configure pushbutton KEYs to generate interrupts
    
    enable_A9_interrupts();  // enable interrupts

    set_pixel_buffers();     // set up the pixel buffers

    init_rand_point ();

    while (1)
    {
        // set new number of boxes
        num_boxes = *SW_ptr;
        if (num_boxes > SW_MAX)
            num_boxes = SW_MAX;

        init_boxes();       // initialize any new boxes
        clear_boxes();      // erase any boxes and lines in the last iteration
        update_old_boxes(); // update the locations of boxes
        move_boxes();
        draw_box_lines();             // draw the boxes and lines

        draw_rand_point ();

        wait_for_vsync(); // swap front and back buffers on VGA vertical sync
        pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
    }
}


/* Draw larger structures */
void init_boxes (){
    for (int i = num_curr_boxes; i < num_boxes; ++i) {
        boxes[i].x = rand() % RESOLUTION_X;
        boxes[i].y = rand() % RESOLUTION_Y;
        boxes[i].x_dir = rand() % 2 * 2 - 1;
        boxes[i].y_dir = rand() % 2 * 2 - 1;
        boxes[i].color = (short int)(rand() % 0xFFFF);
    }
}

void move_boxes (){
    for (int i = 0; i < num_boxes; ++i) {
        if (boxes[i].x <= 0 || boxes[i].x >= RESOLUTION_X)
            boxes[i].x_dir *= -1;
        if (boxes[i].y <= 0 || boxes[i].y >= RESOLUTION_Y)
            boxes[i].y_dir *= -1; 

        boxes[i].x += boxes[i].x_dir;
        boxes[i].y += boxes[i].y_dir;
    }
    num_curr_boxes = num_boxes;
}

void update_old_boxes (){
    for (int i = 0; i < num_curr_boxes; ++i) {
        old_boxes[i].x = boxes[i].x;
        old_boxes[i].y = boxes[i].y;
    }
    num_old_boxes = num_curr_boxes;
}

void clear_boxes() {
    for (int i = 0; i < num_old_boxes; ++i) {
        draw_box(old_boxes[i].x, old_boxes[i].y, BLACK, BOX_LEN);
        plot_line(old_boxes[i].x, old_boxes[i].y, 
                  old_boxes[(i+1)%num_old_boxes].x, old_boxes[(i+1)%num_old_boxes].y, BLACK);
    }
}

void draw_box_lines() {
    for (int i = 0; i < num_curr_boxes; ++i) {
        draw_box(boxes[i].x, boxes[i].y, boxes[i].color, BOX_LEN);
        plot_line(boxes[i].x, boxes[i].y, 
                  boxes[(i+1)%num_curr_boxes].x, boxes[(i+1)%num_curr_boxes].y, 
                  boxes[i].color);
    }
}


/* Draw simple shapes and lines */
void plot_pixel(int x, int y, short int line_color)
{
    *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = line_color;
}

void draw_box(int x0, int y0, short int color, int length)
{
    for(int i = 0; i < length; ++i) {
        for(int j = 0; j < length; ++j) {
            plot_pixel(x0 + i, y0 + j, color);
        }
    }
}

// line drawing using Bresenham's algorithm
void plot_line(int x0, int y0, int x1, int y1, short int line_color)
{
    int is_steep = (ABS(y1 - y0) > ABS(x1 - x0)) ? TRUE : FALSE;

    if (is_steep) {
        swap(&x0, &y0);
        swap(&x1, &y1);
    }
    if (x0 > x1) {
        swap(&x0, &x1);
        swap(&y0, &y1);
    }
    
    int delta_x = x1 - x0;
    int delta_y = ABS(y1 - y0);
    int error = -(delta_x / 2);
    int y = y0;
    int y_step = (y0 < y1) ? 1 : -1;

    for (int x = x0; x < x1; ++x) {
        if (is_steep)
            plot_pixel(y, x, line_color);
        else
            plot_pixel(x, y, line_color);
        error = error + delta_y;
        if (error > 0) {
            y = y + y_step;
            error = error - delta_x;
        }
    }
}

// ellipse using algorithm from https://zingl.github.io/Bresenham.pdf
void plot_ellipse(int x0, int y0, int r1, int r2, short int color) 
{
    long x = -r1, y = 0;                      /* II. quadrant from bottom left to top right */
    long e2 = r2, dx = (1 + 2 * x) * e2 * e2; /* error increment */
    long dy = x * x, err = dx + dy;          /* error of 1.step */

    do
    {
        // loop to fill in the ellipse
        for (int counter = 0; counter <= abs(x); counter++) {
            plot_pixel(x0 - counter, y0 + y, color); // Quadrant 1
            plot_pixel(x0 + counter, y0 + y, color); // Quadrant 2
            plot_pixel(x0 + counter, y0 - y, color); // Quadrant 3
            plot_pixel(x0 - counter, y0 - y, color); // Quadrant 4
        }

        e2 = 2 * err;

        // x step
        if (e2 >= dx)
        {
            x++;
            err += dx += 2 * (long)r2 * r2;
        } 

        // y step
        if (e2 <= dy)
        {
            y++;
            err += dy += 2 * (long)r1 * r1;
        }
    } while (x <= 0);

    while (y++ < r2)
    {
        plot_pixel(x0, y0 + y, color); /* -> finish tip of ellipse */
        plot_pixel(x0, y0 - y, color);
    }
}

// quadratic Bezier curve (for a limited quadratic Bezier segment)
// drawing using algorithm from https://zingl.github.io/Bresenham.pdf
void plot_quad_bezier_seg(int x0, int y0, int x1, int y1, int x2, int y2, short int line_color)
{
    // relative values for checks
    int sx = x2 - x1, sy = y2 - y1;
    long xx = x0 - x1, yy = y0 - y1, xy;
    double dx, dy, err, cur = xx * sy - yy * sx; // curvature
    
    assert (xx * sx <= 0 && yy * sy <= 0); // sign of gradient must not change
    
    if (sx * (long)sx + sy * (long)sy > xx * xx + yy * yy) { // begin with longer part
        x2 = x0; x0 = sx + x1; 
        y2 = y0; y0 = sy + y1; 
        cur = -cur; // swap P0 P2
    }
    if (cur != 0) { // no straight line
        xx += sx; yy += sy;
        sx = x0 < x2 ? 1 : -1; sy = y0 < y2 ? 1 : -1;
        xx *= sx; // x step direction
        yy *= sy; // y step direction

        xy = 2 * xx * yy; // differences 2nd degree
        xx *= xx; yy *= yy;

        if (cur * sx * sy < 0) { // negated curvature?
            xx = -xx; yy = -yy; xy = -xy; cur = -cur;
        }

        dx = 4.0 * sy * cur * (x1 - x0) + xx - xy; // differences 1st degree
        dy = 4.0 * sx * cur * (y0 - y1) + yy - xy;
        xx += xx; yy += yy; err = dx + dy + xy; // error 1st step

        do {
            plot_pixel(x0, y0, line_color); // plot curve
            if (x0 == x2 && y0 == y2) // last pixel -> curve finished
                return; 

            y1 = 2 * err < dx; // save value for test of y step
            if (2 * err > dy) { // x step
                x0 += sx; dx -= xy; err += dy += yy;
            }
            if ( y1 ) { // y step
                y0 += sy; dy -= xy; err += dx += xx; 
            }
        } while (dy < 0 && dx > 0); // gradient negates -> algorithm fails
    }
    plot_line(x0, y0, x2, y2, line_color); /* plot remaining part to end */
}

// quadratic Bezier curve (for any quadratic Bezier segment)
// drawing using algorithm from https://zingl.github.io/Bresenham.pdf
void plot_quad_bezier(int x0, int y0, int x1, int y1, int x2, int y2, short int line_color)
{
    int x = x0 - x1, y = y0 - y1;
    double r, t = x0 - 2 * x1 + x2;

    if ((long)x * (x2 - x1) > 0) { // horizontal cut at P4?
        if ((long)y * (y2 - y1) > 0) // vertical cut at P6 too?
            if (fabs((y0 - 2 * y1 + y2) / t * x) > abs(y)) { // which first?
                x0 = x2; x2 = x + x1; // swap points
                y0 = y2; y2 = y + y1;
            } // now horizontal cut at P4 comes first
        t = (x0 - x1) / t;
        r = (1 - t) * ((1 - t) * y0 + 2.0 * t * y1) + t * t * y2; // By(t = P4)
        t = (x0 * x2 - x1 * x1) * t / (x0 - x1); // gradient dP4/dx = 0
        x = floor(t + 0.5); y = floor(r + 0.5);
        r = (y1 - y0) * (t - x0) / (x1 - x0) + y0; // intersect P3 | P0 P1
        
        plot_quad_bezier_seg(x0, y0, x, floor(r+0.5), x, y, line_color);
        r = (y1 - y2) * (t - x2) / (x1 - x2) + y2; // intersect P4 | P1 P2
        x0 = x; x1 = x; // P0 = P4
        y0 = y; y1 = floor(r + 0.5); // P1 = P8
    }
    if ((long)(y0-y1) * (y2-y1) > 0) { // vertical cut at P6?
        t = y0 - 2 * y1 + y2; t = (y0 - y1) / t;
        r = (1 - t) * ((1 - t) * x0 + 2.0 * t * x1) + t * t * x2; // Bx(t = P6)
        t = (y0 * y2 - y1 * y1) * t / (y0 - y1); // gradient dP6/dy = 0
        x = floor(r + 0.5); y = floor(t + 0.5);
        r = (x1 - x0) * (t - y0) / (y1 - y0) + x0; // intersect P6 | P0 P1

        plot_quad_bezier_seg(x0, y0, floor(r+0.5), y, x, y, line_color);
        r = (x1 - x2) * (t - y2) / (y1 - y2) + x2; // intersect P7 | P1 P2
        x0 = x; x1 = floor(r + 0.5); // P0 = P6, P1 = P7
        y0 = y; y1 = y; // P1 = P7
    }
    plot_quad_bezier_seg(x0, y0, x1, y1, x2, y2, line_color); // remaining part
}

void clear_screen()
{
    for (int x = 0; x < RESOLUTION_X; ++x) {
        for (int y = 0; y < RESOLUTION_Y; ++y) {
            *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = BLACK;
        }
    }
}


/* Helper functions */

// Swaps the first and second integers using a temporary variable
void swap (int *first, int *second) {
    int temp = *first;
    *first = *second;
    *second = temp;
}


/* Set up and manage pixel buffers for VGA drawing */
void set_pixel_buffers(void)
{
    // Pixel buffer base address
    volatile int * pixel_ctrl_ptr = (int *)PIXEL_BUF_CTRL_BASE;

    /* Set front pixel buffer to start of FPGA On-chip memory */
    *(pixel_ctrl_ptr + 1) = FPGA_ONCHIP_BASE; // first store the address in the 
                                              // back buffer

    /* Swap the front/back buffers, to set the front buffer location */
    wait_for_vsync();

    /* Initialize a pointer to the pixel buffer, used by drawing functions */
    pixel_buffer_start = *pixel_ctrl_ptr;
    clear_screen(); // pixel_buffer_start points to the pixel buffer

    /* Set back pixel buffer to start of SDRAM memory */
    *(pixel_ctrl_ptr + 1) = SDRAM_BASE;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // we draw on the back buffer
    clear_screen(); // pixel_buffer_start points to the pixel buffer
}

void wait_for_vsync(){
	volatile int * pixel_ctrl_ptr = (int *)PIXEL_BUF_CTRL_BASE; 	// pixel controller (DMA base)
	register int status;
	
	// start the synchronization process
	*pixel_ctrl_ptr = 1;				// writing to buffer register
	status = *(pixel_ctrl_ptr + 3);		// read status register
	while ((status & 0x01) != 0)		// checking if S bit is 0
		status = *(pixel_ctrl_ptr + 3); // update read value of status register
}


/* Interrupt service routines */
void interval_timer_ISR(void)
{
    // interal timer base address
    volatile int * interval_timer_ptr = (int *)TIMER_BASE;
    *(interval_timer_ptr) = 0; // clear the interrupt

    ++gTime;
}

void pushbutton_ISR(void) 
{
    // KEY base address
    volatile int * KEY_ptr = (int *) KEY_BASE;
    
    int press = *(KEY_ptr + 3); // read the pushbutton interrupt register
    *(KEY_ptr + 3) = press; // clear the interrupt

    if (press & 0x1); // KEY0
    else if (press & 0x2); // KEY1
    else if (press & 0x4); // KEY2
    else; // press & 0x8, which is KEY3
}


/* Set up interrupts, 
 * from the Intel® FPGA University Program DE1-SoC Computer Manual */

// Initialize the banked stack pointer register for IRQ mode
void set_A9_IRQ_stack(void)
{
    int stack, mode;
    stack = A9_ONCHIP_END - 7; // top of A9 onchip memory, aligned to 8 bytes
    /* change processor to IRQ mode with interrupts disabled */
    mode = INT_DISABLE | IRQ_MODE;
    asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
    /* set banked stack pointer */
    asm("mov sp, %[ps]" : : [ps] "r"(stack));

    /* go back to SVC mode before executing subroutine return */
    mode = INT_DISABLE | SVC_MODE;
    asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
}

// Configure the Generic Interrupt Controller (GIC)
void config_GIC(void)
{
    int address; // used to calculate register addresses

    /* configure the FPGA interval timer and KEYs interrupts */
    *((int *)0xFFFED848) = 0x00000101;
    *((int *)0xFFFED108) = 0x00000300;

    // Set Interrupt Priority Mask Register (ICCPMR) and
    // Enable interrupts of all priorities
    address = MPCORE_GIC_CPUIF + ICCPMR;
    *((int *)address) = 0xFFFF;

    // Set CPU Interface Control Register (ICCICR)
    // Enable signaling of interrupts
    address = MPCORE_GIC_CPUIF + ICCICR;
    *((int *)address) = ENABLE;

    // Configure the Distributor Control Register (ICDDCR) to send pending
    // interrupts to CPUs
    address = MPCORE_GIC_DIST + ICDDCR;
    *((int *)address) = ENABLE;
}

// Setup the interval timer interrupts in the FPGA
void config_interval_timer(void)
{
    // interal timer base address
    volatile int * interval_timer_ptr = (int *)TIMER_BASE;

    /* set the interval timer period */
    int counter = TIMER_LOAD; // 1/(100 MHz) x 5x10^6 = 50 msec
    *(interval_timer_ptr + 2) = (counter & 0xFFFF);
    *(interval_timer_ptr + 3) = (counter >> 16) & 0xFFFF;

    /* start interval timer, enable its interrupts */
    // STOP = 0, START = 1, CONT = 1, ITO = 1
    *(interval_timer_ptr + 1) = 0x7;
}

// Setup the KEY interrupts in the FPGA
void config_KEYs(void)
{
    volatile int * KEY_ptr = (int *)KEY_BASE; // pushbutton KEY address
    *(KEY_ptr + 2) = 0xF; // enable interrupts for all KEYs
}

// Turn on interrupts in the ARM processor
void enable_A9_interrupts(void)
{
    int status = SVC_MODE | INT_ENABLE;
    asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}


/* Exception handlers, 
 * from the Intel® FPGA University Program DE1-SoC Computer Manual */

// Define the IRQ exception handler
void __attribute__((interrupt)) __cs3_isr_irq(void)
{
    // Read the ICCIAR from the processor interface
    int address = MPCORE_GIC_CPUIF + ICCIAR;
    int interrupt_ID = *((int *)address);

    // Check which device is calling the interrupt
    if (interrupt_ID == INTERVAL_TIMER_IRQ)
        interval_timer_ISR();
    else if (interrupt_ID == KEYS_IRQ)
        pushbutton_ISR();
    else
        while (1); // if unexpected, then stay here
    
    // Write to the End of Interrupt Register (ICCEOIR)
    address = MPCORE_GIC_CPUIF + ICCEOIR;
    *((int *)address) = interrupt_ID;
}

// Define the remaining exception handlers
void __attribute__((interrupt)) __cs3_reset(void)
{
    while (1);
}

void __attribute__((interrupt)) __cs3_isr_undef(void)
{
    while (1);
}

void __attribute__((interrupt)) __cs3_isr_swi(void)
{
    while (1);
}

void __attribute__((interrupt)) __cs3_isr_pabort(void)
{
    while (1);
}

void __attribute__((interrupt)) __cs3_isr_dabort(void)
{
    while (1);
}

void __attribute__((interrupt)) __cs3_isr_fiq(void)
{
    while (1);
}
