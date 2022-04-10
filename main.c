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
#define WHITE                 0xFFFF
#define YELLOW                0xFFE0
#define RED                   0xF800
#define GREEN                 0x07E0
#define BLUE                  0x001F
#define CYAN                  0x07FF
#define MAGENTA               0xF81F
#define GREY                  0xC618
#define PINK                  0xFC18
#define ORANGE                0xFC00
#define BLACK                 0x0000

#define ABS(x) (((x) > 0) ? (x) : -(x))

/* Screen size */
#define RESOLUTION_X          320
#define RESOLUTION_Y          240
#define INVALID_POS           -1

/* Constants for animation */
#define BOX_LEN 2
#define NUM_BOXES 8
#define SW_MAX 64

#define FALSE 0
#define TRUE 1

/* Constants for moving points and vector field */
#define FIELD_SCALE 60 // the higher, the less the field changes
#define SPEED_SCALE 0.5

/* Constants for stem growth */
#define ADD_NODE_PERIOD 30 // 1 new node per 30*0.1 seconds
#define BEZ_CHANGE_FREQ 12 // 12 Bezier control point changes per growth
#define BEZ_DIV_LIM 2 // allow enough distance before determining the Bezier control points


/* Include directives */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


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

typedef struct Flower_Info {
    int x;                  // (x,y) positions
    int y;
    int x_dir;              // directions, either -1 or 1
    int y_dir;
    short int color;        // colour
    short int size;         // size (number of petals)
} Flower_Info;

typedef struct Rand_Flower_Info {
    double x;               // (x,y) positions
    double y;
    double x_dir;           // directions, either -1 or 1
    double y_dir;
    short int color;        // colour
    short int size;         // size (number of petals)
    int reverse_x;
    int reverse_y;
} Rand_Flower_Info;

typedef struct Moving_Point {
    double x;               // (x,y) positions
    double y;
    double x_dir;           // velocities
    double y_dir;
    short int color;        // colour
    short int reverse_x;    // reverses the velocities
    short int reverse_y;
} Moving_Point;

// used to return 2D points with doubles
typedef struct D_Point {
    double x;               // (x,y) positions
    double y;
} D_Point;

typedef struct Stem_Node {
    Moving_Point point; // the node itself
    Moving_Point bez_ctr_point; // used as Bezier control point between
                                // this node and previous node
    D_Point old_bez_point; // previous location of Bezier control point
    D_Point old_point; // previous location of node
    int creation_time; // the time at which this node is created
    int bez_ctr_time; // the time the Bezier control point is established
    struct Stem_Node * next;
} Stem_Node;

typedef struct Stem_List {
    short int is_growing; // either 0 or 1, if stem currently growing at tail
    short int created_new; // either 0 or 1, ensures only one new node at a time
    int length;
    int growth_period;
    D_Point grow_ctr_points[BEZ_CHANGE_FREQ - BEZ_DIV_LIM]; // points for growing node bezier curve
    short int color;
    Stem_Node * head;
    Stem_Node * tail;
} Stem_List;


/*
 * GLOBAL VARIABLES
 */
int gTime;

Box_Info boxes[SW_MAX];
Box_Info old_boxes[SW_MAX];

int num_boxes;
int num_curr_boxes;
int num_old_boxes;

Flower_Info flowers[SW_MAX];
Flower_Info old_flowers[SW_MAX];

int num_flowers;
int num_curr_flowers;
int num_old_flowers;

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

/* Initialize structures */
void init_moving_point (Moving_Point *point, double x, double y, short int rev_x, short int rev_y);
void init_stem_node (Stem_Node *node, double x, double y, double bez_x, double bez_y, short int rev_x, short int rev_y);
void init_stem_list (Stem_List *list, double x, double y, int color);

/* Operate on stem list */
void add_stem_node (Stem_List *list, double x, double y, double bez_x, double bez_y, short int rev_x, short int rev_y);
void free_stem_list (Stem_List *list);

/* Draw stem */
D_Point vector_field (double x, double y, double time);

/* Draw larger structures */
void init_boxes();
void move_boxes();
void update_old_boxes();
void clear_boxes();
void draw_box_lines();

/* Draw and animate flowers */
void init_flowers();
void move_flowers();
void update_old_flowers();
void clear_flowers();
void draw_all_flowers();
void draw_flower(int x, int y, short int color, short int size);

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
// returns the control point for the Bezier curve described by the three point parameters
D_Point bez_ctr_from_curve (double x0, double y0, double xt, double yt, double x2, double y2, double tr);


Stem_List test_stem;


void init_stem () 
{
    int start_x = rand() % RESOLUTION_X;
    int start_y = rand() % RESOLUTION_Y;

    init_stem_list(&test_stem, start_x, start_y, (short int)(rand() % 0xFFFF));
    
    add_stem_node(&test_stem, start_x, start_y, INVALID_POS, INVALID_POS, 1, 1);
}

void draw_stem () 
{
    //// add new nodes
    /* IF IS STILL GROWING */
    if (test_stem.created_new == FALSE && (gTime - test_stem.tail->creation_time) >= test_stem.growth_period) {
        add_stem_node(&test_stem, test_stem.tail->old_point.x, test_stem.tail->old_point.y, 
                      test_stem.tail->old_point.x, test_stem.tail->old_point.y, 
                      test_stem.tail->point.reverse_x, test_stem.tail->point.reverse_y);
        /* UPDATE STEM GROWTH PERIOD */
        test_stem.created_new = TRUE;
    } else {
        test_stem.created_new = FALSE;
    }
    
    //// clear stem
    Stem_Node *n = test_stem.head;
	while (n != NULL && n->next != NULL) { // traverse list until reach last node
		draw_box((int)(n->old_point).x, (int)(n->old_point).y, BLACK, 2);
        //draw_box((int)(n->next->bez_ctr_point).x, (int)(n->next->old_bez_point).y, BLACK, 4); /* TESTING BEZIER CTR POINT */
        draw_box((int)(n->next->old_point).x, (int)(n->next->old_point).y, BLACK, 2);
        plot_quad_bezier((int)(n->old_point).x, (int)(n->old_point).y, 
                         (int)(n->next->old_bez_point).x, (int)(n->next->old_bez_point).y,
                         (int)(n->next->old_point).x, (int)(n->next->old_point).y,
                         BLACK);
        n = n->next; // n points to last node of list when done
    }

    //// update old points
    n = test_stem.head;
	while (n != NULL) {
        n->old_point.x = n->point.x;
        n->old_point.y = n->point.y;
        n->old_bez_point.x = n->bez_ctr_point.x;
        n->old_bez_point.y = n->bez_ctr_point.y;

        n = n->next;
    }
    
    //// move points of stem
    n = test_stem.head;
	while (n != NULL) {
        if (n->point.x <= 0 || n->point.x >= RESOLUTION_X)
            n->point.reverse_x *= -1;
        if (n->point.y <= 0 || n->point.y >= RESOLUTION_Y)
            n->point.reverse_y *= -1; 
        n->point.x += SPEED_SCALE * n->point.x_dir * n->point.reverse_x;
        n->point.y += SPEED_SCALE * n->point.y_dir * n->point.reverse_y;

        if (n->point.x < 0) n->point.x = 0;
        if (n->point.y < 0) n->point.y = 0;

        // change direction
        double dir_factor = (n == test_stem.tail) ? 1 : 0.1;
        D_Point dir = vector_field(n->point.x, n->point.y, gTime);
        n->point.x_dir = dir_factor * dir.x;
        n->point.y_dir = dir_factor * dir.y;

        n = n->next;
    }

    //// move Bezier control points
    n = test_stem.head;
	while (n != NULL && n->next != NULL) { // traverse list until reach last node
		if (n->bez_ctr_point.x <= 0 || n->bez_ctr_point.x >= RESOLUTION_X)
            n->bez_ctr_point.reverse_x *= -1;
        if (n->bez_ctr_point.y <= 0 || n->bez_ctr_point.y >= RESOLUTION_Y)
            n->bez_ctr_point.reverse_y *= -1; 
        n->bez_ctr_point.x += SPEED_SCALE * n->bez_ctr_point.x_dir * n->bez_ctr_point.reverse_x;
        n->bez_ctr_point.y += SPEED_SCALE * n->bez_ctr_point.y_dir * n->bez_ctr_point.reverse_y;

        if (n->bez_ctr_point.x < 0) n->bez_ctr_point.x = 0;
        if (n->bez_ctr_point.y < 0) n->bez_ctr_point.y = 0;

        // change direction
        double dir_factor = (n == test_stem.tail) ? 1 : 0.1;
        D_Point dir = vector_field(n->bez_ctr_point.x, n->bez_ctr_point.y, gTime);
        n->bez_ctr_point.x_dir = dir_factor * dir.x;
        n->bez_ctr_point.y_dir = dir_factor * dir.y;

        n = n->next;
    }

    /// set new Bezier control point for growing point
    int d_node_time = gTime - test_stem.tail->creation_time; // time the node has existed for
    int bez_time_div = test_stem.growth_period/BEZ_CHANGE_FREQ;

    // determine stored growth point position values for Bezier control point in stem
    for (int i = BEZ_CHANGE_FREQ; i > BEZ_DIV_LIM; --i) {
        if (test_stem.head == test_stem.tail)
            break;
        if (d_node_time > i*(bez_time_div/2))
            break;
        if (d_node_time < (i-1)*(bez_time_div/2)) // if less than previous division, skip
            continue;
        test_stem.grow_ctr_points[i - 1 - BEZ_DIV_LIM].x = test_stem.tail->point.x;
        test_stem.grow_ctr_points[i - 1 - BEZ_DIV_LIM].y = test_stem.tail->point.y;
    }

    // use stored values to calculate Bezier control point for growth point
    for (int i = BEZ_CHANGE_FREQ; i > BEZ_DIV_LIM; --i) {
        if (test_stem.head == test_stem.tail)
            break;
        if (d_node_time > i*bez_time_div) // if more than current division, done
            break;
        if (d_node_time < (i-1)*bez_time_div) // if less than previous division, skip
            continue;
        
        Stem_Node *pre_tail = test_stem.head;
        while (pre_tail != NULL && pre_tail->next != test_stem.tail) {
            pre_tail = pre_tail->next;
        }

        double time_ratio = (double)(i*(bez_time_div/2)) / (double)d_node_time;

        D_Point new_bez_point = bez_ctr_from_curve(pre_tail->point.x, pre_tail->point.y,
                                                   test_stem.grow_ctr_points[i - 1 - BEZ_DIV_LIM].x, test_stem.grow_ctr_points[i - 1 - BEZ_DIV_LIM].y,
                                                   test_stem.tail->point.x, test_stem.tail->point.y, time_ratio);
        test_stem.tail->bez_ctr_point.x = new_bez_point.x;
        test_stem.tail->bez_ctr_point.y = new_bez_point.y;
        test_stem.tail->bez_ctr_time = gTime;
    }


    //// draw stem
    n = test_stem.head;
	while (n != NULL && n->next != NULL) { // traverse list until reach last node
		draw_box((int)(n->point).x, (int)(n->point).y, test_stem.color, 2);
        draw_box((int)(n->next->point).x, (int)(n->next->point).y, test_stem.color, 2);
        plot_quad_bezier((int)(n->point).x, (int)(n->point).y, 
                         (int)(n->next->bez_ctr_point).x, (int)(n->next->bez_ctr_point).y,
                         (int)(n->next->point).x, (int)(n->next->point).y,
                         test_stem.color);
        n = n->next;
    }
}

Moving_Point rand_box;
Moving_Point old_rand_box;
D_Point initial_point;

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
	draw_box((int)old_rand_box.x - 9, (int)old_rand_box.y - 9, 
             old_rand_box.color, 18);
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

    //draw_box((int)rand_box.x, (int)rand_box.y, rand_box.color, 5);
    draw_flower((int)rand_box.x, (int)rand_box.y, rand_box.color, 5);
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

    // initialize number of flowers
    num_curr_flowers = 0;
    num_flowers = *SW_ptr;
    if (num_flowers > SW_MAX)
        num_flowers = SW_MAX;

    // initialize location and direction of rectangles
    init_boxes();

    // initialize location and direction of flowers
    init_flowers();

    set_A9_IRQ_stack();      // initialize the stack pointer for IRQ mode
    config_GIC();            // configure the general interrupt controller
    config_interval_timer(); // configure interval timer to generate interrupts
    config_KEYs();           // configure pushbutton KEYs to generate interrupts
    
    enable_A9_interrupts();  // enable interrupts

    set_pixel_buffers();     // set up the pixel buffers

    init_stem ();

    while (1)
    {
        // set new number of boxes
        num_boxes = *SW_ptr;
        if (num_boxes > SW_MAX)
            num_boxes = SW_MAX;

        // set new number of flowers
        num_flowers = *SW_ptr;
        if (num_flowers > SW_MAX)
            num_flowers = SW_MAX;

        init_boxes();       // initialize any new boxes
        clear_boxes();      // erase any boxes and lines in the last iteration
        update_old_boxes(); // update the locations of boxes
        move_boxes();
        draw_box_lines();             // draw the boxes and lines

        init_flowers();         // initialize any new flowers
        clear_flowers();        // erase any flowers in the last iteration
        update_old_flowers();   // update the locations of flowers
        move_flowers();         // move the flowers
        draw_all_flowers();     // draw all of the flowers

        draw_rand_point ();
        draw_stem ();

        wait_for_vsync(); // swap front and back buffers on VGA vertical sync
        pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
    }
}


/* Initialize structures */
void init_moving_point (Moving_Point *point, double x, double y, short int rev_x, short int rev_y) 
{
    D_Point dir = vector_field(x, y, gTime);
    point->x = x;
    point->y = y;
    point->x_dir = dir.x;
    point->y_dir = dir.y;
    point->reverse_x = rev_x;
    point->reverse_y = rev_y;
}

void init_stem_node (Stem_Node *node, double x, double y, double bez_x, double bez_y, short int rev_x, short int rev_y) 
{
    init_moving_point(&(node->point), x, y, rev_x, rev_y);
    init_moving_point(&(node->bez_ctr_point), bez_x, bez_y, rev_x, rev_y);
    (node->old_point).x = x;
    (node->old_point).y = y;
    (node->old_bez_point).x = bez_x;
    (node->old_bez_point).y = bez_y;
    node->creation_time = gTime;
    node->bez_ctr_time = gTime;
    node->next = NULL;
}

void init_stem_list (Stem_List *list, double x, double y, int color) 
{
    list->is_growing = TRUE;
    list->created_new = TRUE;
    list->length = 0;
    list->growth_period = ADD_NODE_PERIOD;       /*** needs function to determine this!!!!!! ***/
    for (int i = 0; i < BEZ_CHANGE_FREQ - BEZ_DIV_LIM; ++i) {
        (list->grow_ctr_points[i]).x = x;
        (list->grow_ctr_points[i]).y = y;
    }
    list->color = color;
    list->head = NULL;
    list->tail = NULL;
}

/* Operate on stem list */
void add_stem_node (Stem_List *list, double x, double y, double bez_x, double bez_y, short int rev_x, short int rev_y) 
{
    Stem_Node * new_node = (Stem_Node*) malloc(sizeof(Stem_Node));
    if (new_node == NULL) // check if enough memory to allocate
        return;
    
    list->length += 1; // adding stem node increases length of stem list
    init_stem_node(new_node, x, y, bez_x, bez_y, rev_x, rev_y);
    
    if (list->head == NULL) { // list is empty, create first node
        list->head = new_node;
        list->tail = new_node;
        return;
    }

    list->tail->next = new_node; // attach new node to end of list
    list->tail = list->tail->next; // update tail of list
}

void free_stem_list (Stem_List *list) 
{
    while(list->head != NULL) {
		Stem_Node *new_head = list->head->next; // node after head
	    free(list->head); // free memory used by current head
        list->head = new_head;
	}
    list->length = 0;
	list->head = NULL; // list is now empty
    list->tail = NULL;
}


/* Draw stem */
D_Point vector_field (double x, double y, double time) {
    D_Point point;
    //point.x = 0.5 * sin((x + y)/FIELD_SCALE) - cos(time/(FIELD_SCALE));
    //point.y = - 0.5 * cos((x + y)/FIELD_SCALE) + sin(time/(FIELD_SCALE));
    point.x = sin((sin(time/FIELD_SCALE/4)*x + sin(time/FIELD_SCALE)*y)/FIELD_SCALE);
    point.y = -cos((cos(time/FIELD_SCALE)*x + cos(time/FIELD_SCALE/4)*y)/FIELD_SCALE);
    return point;
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

/* Draw and animate flowers */
void init_flowers (){
    for (int i = num_curr_flowers; i < num_flowers; ++i) {
        flowers[i].x = rand() % RESOLUTION_X;
        flowers[i].y = rand() % RESOLUTION_Y;
        flowers[i].x_dir = rand() % 2 * 2 - 1;
        flowers[i].y_dir = rand() % 2 * 2 - 1;
        flowers[i].color = (short int)(rand() % 0xFFFF);
        flowers[i].size = 0;
    }
}

void move_flowers (){
    for (int i = 0; i < num_flowers; ++i) {
        if (flowers[i].x <= 0 || flowers[i].x >= RESOLUTION_X)
            flowers[i].x_dir *= -1;
        if (flowers[i].y <= 0 || flowers[i].y >= RESOLUTION_Y)
            flowers[i].y_dir *= -1; 

        flowers[i].x += flowers[i].x_dir;
        flowers[i].y += flowers[i].y_dir;
    }
    num_curr_flowers = num_flowers;
}

void update_old_flowers (){
    for (int i = 0; i < num_curr_flowers; ++i) {
        old_flowers[i].x = flowers[i].x;
        old_flowers[i].y = flowers[i].y;
    }
    num_old_flowers = num_curr_flowers;
}

void clear_flowers() {
    for (int i = 0; i < num_old_flowers; ++i) {
        draw_flower(old_flowers[i].x, old_flowers[i].y, BLACK, old_flowers[i].size);
    }
}

void draw_all_flowers() {
    for (int i = 0; i < num_curr_flowers; ++i) {
        draw_flower(flowers[i].x, flowers[i].y, flowers[i].color, flowers[i].size);
    }
}

void draw_flower(int x, int y, short int color, short int size) {
	int radius = 5;

    // draw the petals
    if (size >= 5) { // fifth layer of petals
        plot_ellipse(x + 7, y + 18, radius + 3, radius + 3, YELLOW);
        plot_ellipse(x - 7, y + 18, radius + 3, radius + 3, YELLOW);
        plot_ellipse(x + 7, y - 18, radius + 3, radius + 3, YELLOW);
        plot_ellipse(x - 7, y - 18, radius + 3, radius + 3, YELLOW);

        plot_ellipse(x + 18, y + 7, radius + 3, radius + 3, YELLOW);
        plot_ellipse(x - 18, y + 7, radius + 3, radius + 3, YELLOW);
        plot_ellipse(x + 18, y - 7, radius + 3, radius + 3, YELLOW);
        plot_ellipse(x - 18, y - 7, radius + 3, radius + 3, YELLOW);
    }

    if (size >= 4) { // fourth layer of petals
        plot_ellipse(x + 3 * radius, y, radius + 1, radius + 1, YELLOW);
        plot_ellipse(x - 3 * radius, y, radius + 1, radius + 1, YELLOW);
        plot_ellipse(x, y + 3 * radius, radius + 1, radius + 1, YELLOW);
        plot_ellipse(x, y - 3 * radius, radius + 1, radius + 1, YELLOW);

        plot_ellipse(x + 2 * radius, y + 2 * radius, radius + 1, radius + 1, YELLOW);
        plot_ellipse(x - 2 * radius, y + 2 * radius, radius + 1, radius + 1, YELLOW);
        plot_ellipse(x + 2 * radius, y - 2 * radius, radius + 1, radius + 1, YELLOW);
        plot_ellipse(x - 2 * radius, y - 2 * radius, radius + 1, radius + 1, YELLOW);
    }

    if (size >= 3) { // third layer of petals
        plot_ellipse(x + 4, y + 2 * radius, radius, radius, YELLOW);
        plot_ellipse(x - 4, y + 2 * radius, radius, radius, YELLOW);
        plot_ellipse(x + 4, y - 2 * radius, radius, radius, YELLOW);
        plot_ellipse(x - 4, y - 2 * radius, radius, radius, YELLOW);

        plot_ellipse(x + 2 * radius, y + 4, radius, radius, YELLOW);
        plot_ellipse(x - 2 * radius, y + 4, radius, radius, YELLOW);
        plot_ellipse(x + 2 * radius, y - 4, radius, radius, YELLOW);
        plot_ellipse(x - 2 * radius, y - 4, radius, radius, YELLOW);
    }

    if (size >= 2) { // second layer of petals
        plot_ellipse(x + 8, y, radius, radius, YELLOW);
        plot_ellipse(x - 8, y, radius, radius, YELLOW);
        plot_ellipse(x, y + 8, radius, radius, YELLOW);
        plot_ellipse(x, y - 8, radius, radius, YELLOW);
    }

    if (size >= 1) { // first layer of petals
        plot_ellipse(x + radius, y + radius, radius, radius, YELLOW);
        plot_ellipse(x - radius, y + radius, radius, radius, YELLOW);
        plot_ellipse(x + radius, y - radius, radius, radius, YELLOW);
        plot_ellipse(x - radius, y - radius, radius, radius, YELLOW);
    }

    // draw center of the flower
    plot_ellipse(x, y, 4, 4, color);
}

/* Draw simple shapes and lines */
void plot_pixel(int x, int y, short int line_color)
{
    if (x < 0 || y < 0 || x > RESOLUTION_X || y > RESOLUTION_Y)
        return;
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

        // black border
        plot_pixel(x0 - abs(x), y0 + y, BLACK); // Quadrant 1
        plot_pixel(x0 + abs(x), y0 + y, BLACK); // Quadrant 2
        plot_pixel(x0 + abs(x), y0 - y, BLACK); // Quadrant 3
        plot_pixel(x0 - abs(x), y0 - y, BLACK); // Quadrant 4

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
    
    if (!(xx * sx <= 0 && yy * sy <= 0)) return; // sign of gradient must not change
    
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

// P0(x0, y0) is the start (t = 0) and P2(x2, y2) is the end (t = 1)
// P(tr) = (xt, yt) is a point on the curve at t = tr, 0 < tr < 1
// returns P1(x1, y1), the control point for the Bezier curve described by the three point parameters
D_Point bez_ctr_from_curve (double x0, double y0, double xt, double yt, double x2, double y2, double tr) 
{
    //P(t) = (1-t)^2 *P0 + 2*(1-t)*t*P1 + t^2 *P2
    double x1 = xt - ((1.0 - tr) * (1.0 - tr) * x0) - (tr * tr * x2);
    x1 /= (2.0 * (1.0 - tr) * tr);
    double y1 = yt - ((1.0 - tr) * (1.0 - tr) * y0) - (tr * tr * y2);
    y1 /= (2.0 * (1.0 - tr) * tr);

    if (x1 > RESOLUTION_X) x1 = RESOLUTION_X;
    if (x1 < 0) x1 = 0;
    if (y1 > RESOLUTION_Y) y1 = RESOLUTION_Y;
    if (y1 < 0) y1 = 0;

    D_Point P1;
    P1.x = x1;
    P1.y = y1;
    return P1;
}

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
