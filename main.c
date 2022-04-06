/* Address values in the DE1-SoC board */

/* Memory */
#define SDRAM_BASE            0xC0000000
#define FPGA_ONCHIP_BASE      0xC8000000
#define FPGA_CHAR_BASE        0xC9000000

/* Cyclone V FPGA devices */
#define LEDR_BASE             0xFF200000
#define HEX3_HEX0_BASE        0xFF200020
#define HEX5_HEX4_BASE        0xFF200030
#define SW_BASE               0xFF200040
#define KEY_BASE              0xFF200050
#define TIMER_BASE            0xFF202000
#define PIXEL_BUF_CTRL_BASE   0xFF203020
#define CHAR_BUF_CTRL_BASE    0xFF203030

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

/* Draw larger structures */
void move_boxes();
void draw();
void draw_boxes();
void clear_boxes();
void init_boxes();
void update_old_boxes();

/* Draw simple shapes and lines */
void draw_line(int x0, int y0, int x1, int y1, short int line_color);
void plot_pixel(int x, int y, short int line_color);
void clear_screen();
void wait_for_vsync();

/* Helper functions */
void swap (int *first, int *second);


/*
 * FUNCTION DEFINITIONS
 */
int main(void)
{
    volatile int * pixel_ctrl_ptr = (int *)0xFF203020;
    volatile int * SW_ptr = (int *)0xFF200040;

    // initialize number of boxes
    num_curr_boxes = 0;
    num_boxes = *SW_ptr;
    if (num_boxes > SW_MAX)
        num_boxes = SW_MAX;

    // initialize location and direction of rectangles
    init_boxes();

    /* set front pixel buffer to start of FPGA On-chip memory */
    *(pixel_ctrl_ptr + 1) = 0xC8000000; // first store the address in the 
                                        // back buffer
    /* now, swap the front/back buffers, to set the front buffer location */
    wait_for_vsync();
    /* initialize a pointer to the pixel buffer, used by drawing functions */
    pixel_buffer_start = *pixel_ctrl_ptr;
    clear_screen(); // pixel_buffer_start points to the pixel buffer
    /* set back pixel buffer to start of SDRAM memory */
    *(pixel_ctrl_ptr + 1) = 0xC0000000;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // we draw on the back buffer
    clear_screen(); // pixel_buffer_start points to the pixel buffer

    while (1)
    {
        // set new number of boxes
        num_boxes = *SW_ptr;
        if (num_boxes > SW_MAX)
            num_boxes = SW_MAX;

        // initialize any new boxes
        init_boxes();
    
        // erase any boxes and lines that were drawn in the last iteration
        clear_boxes();

        // code for updating the locations of boxes
        update_old_boxes();
        move_boxes();

        // code for drawing the boxes and lines
        draw();

        wait_for_vsync(); // swap front and back buffers on VGA vertical sync
        pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
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

void draw() {
    for (int i = 0; i < num_curr_boxes; ++i) {
        draw_box(boxes[i].x, boxes[i].y, boxes[i].color);
        draw_line(boxes[i].x, boxes[i].y, 
                  boxes[(i+1)%num_curr_boxes].x, boxes[(i+1)%num_curr_boxes].y, 
                  boxes[i].color);
    }
}

void clear_boxes() {
    for (int i = 0; i < num_old_boxes; ++i) {
        draw_box(old_boxes[i].x, old_boxes[i].y, 0x0000);
        draw_line(old_boxes[i].x, old_boxes[i].y, 
                  old_boxes[(i+1)%num_old_boxes].x, old_boxes[(i+1)%num_old_boxes].y, 0x0000);
    }
}

void init_boxes (){
    for (int i = num_curr_boxes; i < num_boxes; ++i) {
        boxes[i].x = rand() % RESOLUTION_X;
        boxes[i].y = rand() % RESOLUTION_Y;
        boxes[i].x_dir = rand() % 2 * 2 - 1;
        boxes[i].y_dir = rand() % 2 * 2 - 1;
        boxes[i].color = (short int)(rand() % 0xFFFF);
    }
}

void wait_for_vsync(){
	volatile int * pixel_ctrl_ptr = (int *)0xFF203020; 	// pixel controller (DMA base)
	register int status;
	
	// start the synchronization process
	*pixel_ctrl_ptr = 1;				// writing to buffer register
	status = *(pixel_ctrl_ptr + 3);		// read status register
	while ((status & 0x01) != 0)		// checking if S bit is 0
		status = *(pixel_ctrl_ptr + 3); // update read value of status register
}

void clear_screen()
{
    for (int x = 0; x < RESOLUTION_X; ++x) {
        for (int y = 0; y < RESOLUTION_Y; ++y) {
            *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = 0x0000;
        }
    }
}

void swap (int *first, int *second) {
    int temp = *first;
    *first = *second;
    *second = temp;
}

void plot_pixel(int x, int y, short int line_color)
{
    *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = line_color;
}

void draw_box(int x0, int y0, short int color)
{
    for(int i = 0; i < BOX_LEN; ++i) {
        for(int j = 0; j < BOX_LEN; ++j) {
            plot_pixel(x0 + i, y0 + j, color);
        }
    }
}

void draw_line(int x0, int y0, int x1, int y1, short int line_color)
{
    int is_steep;
    if (ABS(y1 - y0) > ABS(x1 - x0))
        is_steep = TRUE;
    else
        is_steep = FALSE;

    if (is_steep) {
        swap(&x0, &y0);
        swap(&x1, &y1);
    }
    if (x0 > x1) {
        swap(&x0, &x1);
        swap(&y0, &y1);
    }
    
    int deltax = x1 - x0;
    int deltay = ABS(y1 - y0);
    int error = -(deltax / 2);
    int y = y0;

    int y_step;

    if (y0 < y1)
        y_step = 1;
    else
        y_step = -1;

    for (int x = x0; x < x1; ++x) {
        if (is_steep)
            plot_pixel(y, x, line_color);
        else
            plot_pixel(x, y, line_color);
        error = error + deltay;
        if (error > 0) {
            y = y + y_step;
            error = error - deltax;
        }
    }
}
