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
#define PS2_IRQ               79

/* Load value for A9 Private Timer */
#define TIMER_LOAD            10000000       // 1/(100 MHz) x 1x10^7 = 0.1 sec

/* PS/2 Port */
#define PS2_BASE              0xFF200100

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
#define INVALID_VAL           -1

/* Constants for animation */
#define NUM_STARTING_PLANTS 3
#define DEFAULT_SPEED 0.3

/* Constants for moving points and vector field */
#define FIELD_SCALE 70 // the higher, the less the field changes

/* Constants for stem growth */
#define BEZ_CHANGE_FREQ 12 // 12 Bezier control point changes per growth
#define BEZ_DIV_LIM 2 // allow enough distance before determining the Bezier control points

/* Stem length constants */
#define STEM_LEN_BASE 12 // length of a base stem
#define STEM_LEN_REDUC_PER_BRNCH 5 // every time the stem branches, the length is reduced
#define STEM_LEN_DEVIATION 2 // how much the length may deviate from base in either direction

/* Stem branch chance and period constants */
#define STEM_BRNCH_CHANCE_BASE 0.75 // base chance of a stem branching at a node
#define STEM_BRNCH_REDUC_PER_BRNCH 0.25 // every time the stem branches, the chance of further branching reduces
#define STEM_BRNCH_PERIOD_BASE 70 // how long it will take for a node to branch
#define STEM_BRNCH_PERIOD_DEVIATION 20 // how much the branch period may deviate from base in either direction

/* Stem node adding constants */
#define STEM_ADD_PERIOD_BASE 30 // 1 new node per 30*0.1 seconds (must be at least double BEZ_CHANGE_FREQ)
#define STEM_ADD_PERIOD_DEVIATION 20 // how much the period may deviate from base in either direction

/* Stem repelling from other pixels constants */
#define REPEL_RANGE 13 // how far a stem can be affected by repelling
#define REPEL_STRENGTH 1.75 // fastest speed change that repelling can cause

/* Flower growth constants */
#define FLOWER_SIZE_RANGE 21

/* Color constants */
#define MAX_RED_BLUE 0b11111 // 31
#define MAX_GREEN 0b111111 // 63
#define MIN_RGB 0

/* Plant color constants */
#define STEM_MIN_RED 0
#define STEM_MAX_RED 15
#define STEM_MIN_GREEN 31
#define STEM_MAX_GREEN 63
#define STEM_MIN_BLUE 0
#define STEM_MAX_BLUE 10
#define DARK_COLOR_CHANGE 4
#define DARK_CENTER 0x1860
#define YELLOW_CENTER 0xFFE0
#define BRANCH_COLOR_CHANGE 4

/* Boolean constants */
#define FALSE 0
#define TRUE 1


/* Include directives */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


/*
 * STRUCT DECLARATION
 */
typedef struct Moving_Point Moving_Point;
typedef struct D_Point D_Point;
typedef struct Stem_Node Stem_Node;
typedef struct Stem_List Stem_List;
typedef struct Plant_Node Plant_Node;
typedef struct Plant_List Plant_List;


/*
 * STRUCT DEFINITIONS
 */
typedef struct Moving_Point {
    double x;               // (x,y) positions
    double y;
    double x_dir;           // velocities
    double y_dir;
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
    Stem_Node * next;

    short int branchable;
    int branch_period;
    Stem_List * branching_stem;
} Stem_Node;

typedef struct Stem_List {
    short int created_new; // either 0 or 1, ensures only one new node at a time
    int length; // current length of the stem list
    int length_lim; // limit of stem length
    int growth_period;
    D_Point grow_ctr_points[BEZ_CHANGE_FREQ - BEZ_DIV_LIM]; // points for growing node bezier curve
    int num_branches;
    short int max_flower_size;
    short int curr_flower_size;
    short int old_flower_size;
    short int color;
    short int flower_color;
    short int flower_center_color;
    short int border_color;
    Stem_Node * head;
    Stem_Node * tail;
} Stem_List;

typedef struct Plant_Node {
    Stem_List * stem;
    struct Plant_Node * next;
} Plant_Node;

typedef struct Plant_List {
    Plant_Node * head;
    Plant_Node * tail;
} Plant_List;


/*
 * GLOBAL VARIABLES
 */
int gTime;
int speed, old_speed;
double speed_scale, old_speed_scale;
short int clear_all_flag, pause_program_flag, change_speed_flag, mouse_clicked_flag;
int mouse_x, mouse_y, old_mouse_x, old_mouse_y, PS2_mouse_x, PS2_mouse_y;

volatile int pixel_buffer_start;

Plant_List plants;


/*
 * FUNCTION DECLARATIONs
 */
/* Set up interrupts */
void set_A9_IRQ_stack(void);
void config_GIC(void);
void config_interrupt(int N, int CPU_target);
void config_interval_timer(void);
void config_KEYs(void);
void config_PS2(void);
void enable_A9_interrupts(void);

/* Interrupt service routines */
void interval_timer_ISR(void);
void pushbutton_ISR(void);
void PS2_ISR(void);

/* Modify program from interrupts */
void pause_program ();
void clear_program ();
void change_program_speed ();
void mouse_clicked ();
void draw_mouse ();

/* Set up and manage pixel buffers */
void set_pixel_buffers(void);
void wait_for_vsync();

/* Initialize structures */
void init_start_values ();
void init_moving_point (Moving_Point *point, double x, double y, short int rev_x, short int rev_y);
void init_stem_node (Stem_Node *node, double x, double y, double bez_x, double bez_y, short int rev_x, short int rev_y, short int can_branch);
void init_stem_list (Stem_List *list, double x, double y, short int color, short int flower_color, short int flower_center_color, short int border_color, int num_branches);
int get_stem_length (int num_branches);
short int is_branchable (int num_branches);
int get_stem_growth_period ();
void init_plant_list (Plant_List *list);
void init_plant_node (Plant_Node *node, double x, double y);

/* Operate on stem list */
void add_stem_node (Stem_List *list, double x, double y, double bez_x, double bez_y, short int rev_x, short int rev_y);
void free_stem_list (Stem_List *list);

/* Operate on plant list */
void add_plant_node (Plant_List *list, double x, double y);
void free_plant_list (Plant_List *list);

/* Draw stem */
void draw_stem (Stem_List *list);
void add_new_stem_node (Stem_List *list);
void draw_old_stem_nodes (Stem_List *list, short int color);
void draw_stem_nodes (Stem_List *list, short int color);
void draw_old_stem_flower (Stem_List *list);
void draw_stem_flower (Stem_List *list);
void update_old_stem_points (Stem_List *list);
void move_point_in_vec_field (Moving_Point *point, double dir_factor);
void move_point_from_repel (Moving_Point *point);
D_Point repel_from_surround (int x, int y, double x_dir, double y_dir, int range);
void move_stem_points (Stem_List *list);
void store_grow_position (Stem_List *list, int d_node_time, int bez_time_div);
void set_grow_bez_point (Stem_List *list, int d_node_time, int bez_time_div);
void update_growing_stem_bez_point (Stem_List *list);
void branch_stem (Stem_List *list);
void draw_branching_stems_rec (Stem_List *list);
void update_flower_size (Stem_List *list);
void set_flower_size (Stem_List *list);

/* Draw flower */
void draw_flower(int x, int y, short int petal_color, short int center_color, short int border_color, short int size);

/* Draw simple shapes and lines */
void plot_pixel(int x, int y, short int line_color);
short int get_pixel (int x, int y);
void plot_line(int x0, int y0, int x1, int y1, short int line_color);
void plot_ellipse(int x0, int y0, int r1, int r2, short int color, short int border_color);
void plot_quad_bezier_seg(int x0, int y0, int x1, int y1, int x2, int y2, short int line_color);
void plot_quad_bezier(int x0, int y0, int x1, int y1, int x2, int y2, short int line_color);
void clear_screen ();
void clear_all ();

/* Colour changing functions */
// returns a random colour based on parameters limiting the amount of red, green, and blue, 
// the minimum amount for each is 0, the maximum red and blue input is 31, maximum green input is 63
short int randomize_color (short int min_red, short int max_red, short int min_green, short int max_green, short int min_blue, short int max_blue);
short int randomize_color_change (short int color, short int red_change, short int green_change, short int blue_change);
short int darkify_color (short int color);
short int randomize_flower_color ();
void change_plant_colors (Plant_List *list);
void change_stem_colors (Stem_List *list, short int color, short int flower_color, short int flower_center_color);

/* Helper functions */
void swap (int *first, int *second);
D_Point vector_field (double x, double y, double time);
// returns the control point for the Bezier curve described by the three point parameters
D_Point bez_ctr_from_curve (double x0, double y0, double xt, double yt, double x2, double y2, double tr);
void find_surround (int x, int y, int range, int *left_dist, int *right_dist, int *up_dist, int *down_dist);
double average_surround (int x, int y, int range);


/*
 * FUNCTION DEFINITIONS
 */
int main(void)
{
    volatile int * pixel_ctrl_ptr = (int *)PIXEL_BUF_CTRL_BASE;
    volatile int * SW_ptr = (int *)SW_BASE;

    init_start_values();      // initialize starting global variables

    set_A9_IRQ_stack();       // initialize the stack pointer for IRQ mode
    config_GIC();             // configure the general interrupt controller
    config_interval_timer();  // configure interval timer to generate interrupts
    config_KEYs();            // configure pushbutton KEYs to generate interrupts
    config_PS2();             // configure mouse input (PS2) to generate interrupts
    
    set_pixel_buffers();      // set up the pixel buffers

    enable_A9_interrupts();   // enable interrupts

    init_plant_list(&plants); // initialize linked list of plants

    // start by adding plants to the list
    for (int i = 0; i < NUM_STARTING_PLANTS; i++)
        add_plant_node(&plants, rand() % RESOLUTION_X, rand() % RESOLUTION_Y);

    while (1)
    {
        mouse_clicked();          // update the mouse coordinates
        pause_program();          // KEY1 pressed
        change_program_speed();   // KEY2 pressed
        clear_program();          // KEY0 pressed

        Plant_Node *n = plants.head; // draw all the plants in the plant list
        while (n != NULL) {
            draw_stem (n->stem);
            n = n->next;
        }
        
        draw_mouse();             // draw the mouse at mouse coordinates

        wait_for_vsync(); // swap front and back buffers on VGA vertical sync
        pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
    }
}


/* Initialize structures */
void init_start_values ()
{
    // initialize speed
    speed = 1;
    speed_scale = DEFAULT_SPEED;

    // initialize push button flags
    clear_all_flag = FALSE;
    pause_program_flag = FALSE;
    change_speed_flag = FALSE;
    mouse_clicked_flag = FALSE;

    // initialize mouse position
    mouse_x = 0;
    mouse_y = 0;
    old_mouse_x = 0;
    old_mouse_y = 0;
    PS2_mouse_x = 0;
    PS2_mouse_y = 0;
}

void init_moving_point (Moving_Point *point, double x, double y, short int rev_x, short int rev_y) 
{
    D_Point dir = vector_field(x, y, gTime);
    point->x = x;
    point->y = y;
    point->x_dir = dir.x + speed_scale * (rand() % 3 - 1); // start with slightly different velocity
    point->y_dir = dir.y + speed_scale * (rand() % 3 - 1);
    point->reverse_x = rev_x;
    point->reverse_y = rev_y;
}

void init_stem_node (Stem_Node *node, double x, double y, double bez_x, double bez_y, short int rev_x, short int rev_y, short int can_branch) 
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

    node->branchable = can_branch;
    node->branch_period = STEM_BRNCH_PERIOD_BASE + rand() % (STEM_BRNCH_PERIOD_DEVIATION * 2 + 1) - STEM_BRNCH_PERIOD_DEVIATION;
    node->branching_stem = NULL;
}

void init_stem_list (Stem_List *list, double x, double y, short int color, short int flower_color, short int flower_center_color, short int border_color, int num_branches) 
{
    list->created_new = TRUE;

    list->length = 0;
    list->length_lim = get_stem_length(num_branches);

    list->growth_period = get_stem_growth_period();
    for (int i = 0; i < BEZ_CHANGE_FREQ - BEZ_DIV_LIM; ++i) {
        (list->grow_ctr_points[i]).x = x;
        (list->grow_ctr_points[i]).y = y;
    }
    list->num_branches = num_branches;
    list->max_flower_size = INVALID_VAL;
    list->curr_flower_size = INVALID_VAL;
    list->old_flower_size = INVALID_VAL;

    list->color = color;
    list->flower_color = flower_color;
    list->flower_center_color = flower_center_color;
    list->border_color = border_color;
    list->head = NULL;
    list->tail = NULL;
}

// determines the length of a stem
int get_stem_length (int num_branches)
{
    int length = STEM_LEN_BASE; // base stem length
    length -= STEM_LEN_REDUC_PER_BRNCH * num_branches; // adjust for branching

    // random deviation of length from -2 to +2
    int deviation = rand() % (STEM_LEN_DEVIATION * 2 + 1) - STEM_LEN_DEVIATION;
    length += deviation;

    if (length < 0)
        length = 0;

    return length;
}

short int is_branchable (int num_branches) 
{
    double chance = STEM_BRNCH_CHANCE_BASE - STEM_BRNCH_REDUC_PER_BRNCH * num_branches;
    short int branchable = (((double)(rand() % 101) / 100.0) < chance) ? TRUE : FALSE;
    return branchable;
}

// determines the how soon a new stem node may be added (must be at least double BEZ_CHANGE_FREQ)
int get_stem_growth_period ()
{
    int period = STEM_ADD_PERIOD_BASE; // base stem length

    int deviation = rand() % (STEM_ADD_PERIOD_DEVIATION * 2 + 1) - STEM_ADD_PERIOD_DEVIATION;
    period += deviation;

    if (period < BEZ_CHANGE_FREQ * 2)
        period = BEZ_CHANGE_FREQ * 2;

    return period;
}

void init_plant_list (Plant_List *list)
{
    list->head = NULL;
    list->tail = NULL;
}

void init_plant_node (Plant_Node *node, double x, double y)
{
    Stem_List * new_stem = (Stem_List*) malloc(sizeof(Stem_List));
    if (new_stem == NULL) // check if enough memory to allocate
        return;
    
    short int color = randomize_color(STEM_MIN_RED, STEM_MAX_RED, STEM_MIN_GREEN, STEM_MAX_GREEN, STEM_MIN_BLUE, STEM_MAX_BLUE);
    short int flower_color = randomize_flower_color();
    short int flower_center_color = (rand() % 2 == 0) ? DARK_CENTER : YELLOW_CENTER;
    short int border_color = darkify_color(flower_color);
    
    init_stem_list(new_stem, x, y, color, flower_color, flower_center_color, border_color, 0);
    add_stem_node(new_stem, x, y, INVALID_VAL, INVALID_VAL, 1, 1);

    node->stem = new_stem;
    node->next = NULL;
}


/* Operate on stem list */
void add_stem_node (Stem_List *list, double x, double y, double bez_x, double bez_y, short int rev_x, short int rev_y) 
{
    Stem_Node * new_node = (Stem_Node*) malloc(sizeof(Stem_Node));
    if (new_node == NULL) // check if enough memory to allocate
        return;
    
    list->length += 1; // adding stem node increases length of stem list

    short int branchable = is_branchable(list->num_branches);
    init_stem_node(new_node, x, y, bez_x, bez_y, rev_x, rev_y, branchable);
    
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
    while (list->head != NULL) {
        if (list->head->branching_stem != NULL) // free node's branching stems
            free_stem_list(list->head->branching_stem);
		Stem_Node *new_head = list->head->next; // node after head
	    free(list->head); // free memory used by current head
        list->head = new_head;
	}
    list->length = 0;
	list->head = NULL; // list is now empty
    list->tail = NULL;
}


/* Operate on plant list */
void add_plant_node (Plant_List *list, double x, double y)
{
    Plant_Node *new_node = (Plant_Node*) malloc(sizeof(Plant_Node));
    if (new_node == NULL) // check if enough memory to allocate
        return;
    
    init_plant_node(new_node, x, y);

    if (list->head == NULL) { // list is empty, create first node
        list->head = new_node;
        list->tail = new_node;
        return;
    }

    list->tail->next = new_node; // attach new node to end of list
    list->tail = list->tail->next; // update tail of list
}

void free_plant_list (Plant_List *list)
{
    while (list->head != NULL) {
        free_stem_list(list->head->stem);
        Plant_Node *new_head = list->head->next;
        free(list->head);
        list->head = new_head;
    }
    list->head = NULL;
    list->tail = NULL;
}


/* Draw stem */
void add_new_stem_node (Stem_List *list) 
{
    // if the stem can support more nodes, add more
    if (list->length < list->length_lim) {
        if (list->created_new == FALSE) {
            // if appropriate time has passed or just one node in stem (if is a main branch)
            if ( ((gTime - list->tail->creation_time) >= list->growth_period) || 
                 (list->head == list->tail)){// && list->num_branches == 0)) {

                add_stem_node(list, list->tail->old_point.x, list->tail->old_point.y, 
                            list->tail->old_point.x, list->tail->old_point.y, 
                            list->tail->point.reverse_x, list->tail->point.reverse_y);
                list->growth_period = get_stem_growth_period(); // update stem growth period
                list->created_new = TRUE;
            }
        } else {
            list->created_new = FALSE;
        }
    }
}

void draw_old_stem_nodes (Stem_List *list, short int color) 
{
    Stem_Node *n = list->head;
	while (n != NULL && n->next != NULL) { // traverse list until reach last node
        plot_quad_bezier((int)(n->old_point).x, (int)(n->old_point).y, 
                         (int)(n->next->old_bez_point).x, (int)(n->next->old_bez_point).y,
                         (int)(n->next->old_point).x, (int)(n->next->old_point).y,
                         color);
        n = n->next; // n points to last node of list when done
    }
}

void draw_stem_nodes (Stem_List *list, short int color) 
{
    Stem_Node *n = list->head;
	while (n != NULL && n->next != NULL) { // traverse list until reach last node
        plot_quad_bezier((int)(n->point).x, (int)(n->point).y, 
                         (int)(n->next->bez_ctr_point).x, (int)(n->next->bez_ctr_point).y,
                         (int)(n->next->point).x, (int)(n->next->point).y,
                         list->color); 
        n = n->next;
    }
}

void draw_old_stem_flower (Stem_List *list)
{
    if (list->length < list->length_lim) { return; } // if still growing, no flower

    D_Point point = list->tail->old_point; 
    short int size = list->old_flower_size; 
    if (size < 14)
        plot_ellipse(point.x, point.y, size/6 + 2, size/6 + 2, BLACK, BLACK);
    else if (size < 16)
        draw_flower(point.x, point.y, BLACK, BLACK, BLACK, 1);
    else if (size < 18)
        draw_flower(point.x, point.y, BLACK, BLACK, BLACK, 2);
    else if (size < 22)
        draw_flower(point.x, point.y, BLACK, BLACK, BLACK, 3);
    else if (size < 25)
        draw_flower(point.x, point.y, BLACK, BLACK, BLACK, 4);
    else
        draw_flower(point.x, point.y, BLACK, BLACK, BLACK, 5);
}

void draw_stem_flower (Stem_List *list)
{
    if (list->length < list->length_lim) { return; } // if still growing, no flower

    Moving_Point point = list->tail->point;
    short int size = list->curr_flower_size;
    if (size < 14)
        plot_ellipse(point.x, point.y, size/6 + 2, size/6 + 2, list->flower_color, list->border_color);
    else if (size < 17)
        draw_flower(point.x, point.y, list->flower_color, list->flower_center_color, list->border_color, 1);
    else if (size < 20)
        draw_flower(point.x, point.y, list->flower_color, list->flower_center_color, list->border_color, 2);
    else if (size < 23)
        draw_flower(point.x, point.y, list->flower_color, list->flower_center_color, list->border_color, 3);
    else if (size < 25)
        draw_flower(point.x, point.y, list->flower_color, list->flower_center_color, list->border_color, 4);
    else
        draw_flower(point.x, point.y, list->flower_color, list->flower_center_color, list->border_color, 5);
}

void update_old_stem_points (Stem_List *list) 
{
    Stem_Node *n = list->head;
	while (n != NULL) {
        n->old_point.x = n->point.x;
        n->old_point.y = n->point.y;
        n->old_bez_point.x = n->bez_ctr_point.x;
        n->old_bez_point.y = n->bez_ctr_point.y;

        n = n->next;
    }
}

void move_point_in_vec_field (Moving_Point *point, double dir_factor) 
{
    // if at bounds, reverse direction
    if (point->x <= 0 || point->x >= RESOLUTION_X)
        point->reverse_x *= -1;
    if (point->y <= 0 || point->y >= RESOLUTION_Y)
        point->reverse_y *= -1; 
    point->x += speed_scale * point->x_dir * point->reverse_x;
    point->y += speed_scale * point->y_dir * point->reverse_y;

    // if would be beyond bounds, fix within bounds
    if (point->x < 0) point->x = 0;
    if (point->y < 0) point->y = 0;

    // change direction using vector field
    D_Point dir = vector_field(point->x, point->y, gTime);
    point->x_dir = dir_factor * dir.x;
    point->y_dir = dir_factor * dir.y;
}

void move_point_from_repel (Moving_Point *point)
{
    D_Point repel_dir = repel_from_surround(point->x, point->y, point->x_dir, point->y_dir, REPEL_RANGE);
    point->x_dir += repel_dir.x;
    point->y_dir += repel_dir.y;
}

D_Point repel_from_surround (int x, int y, double x_dir, double y_dir, int range) 
{
    // closest point left, right, up, and down of the given point
    int left_dist, right_dist, up_dist, down_dist;
    find_surround(x, y, range, &left_dist, &right_dist, &up_dist, &down_dist);

    D_Point repel_dir; // direction modifier
    repel_dir.x = 0;
    repel_dir.y = 0;

    repel_dir.x += (abs(left_dist) == 1) ? 0 : REPEL_STRENGTH * ((double)(range - left_dist + 1) / (double)range);
    repel_dir.x -= (abs(right_dist) == 1) ? 0 : REPEL_STRENGTH * ((double)(range - right_dist + 1) / (double)range);

    repel_dir.y += (abs(up_dist) == 1) ? 0 : REPEL_STRENGTH * ((double)(range - up_dist + 1) / (double)range);
    repel_dir.y -= (abs(down_dist) == 1) ? 0 : REPEL_STRENGTH * ((double)(range - down_dist + 1) / (double)range);

    return repel_dir;
}

void move_stem_points (Stem_List *list) 
{
    Stem_Node *n = list->head;
	while (n != NULL) {
        double dir_factor = (list->length < list->length_lim && n == list->tail) ? 1 : 0.1;

        move_point_in_vec_field(&(n->point), dir_factor);
        // if tail still growing, do not move its Bezier control point, but move based on surrounding points
        // if tail stopped growing, move its Bezier control point with the others
        if (n->next != NULL || list->length >= list->length_lim)
            move_point_in_vec_field(&(n->bez_ctr_point), dir_factor);
        else
            move_point_from_repel(&(list->tail->point));

        n = n->next;
    }
}

void store_grow_position (Stem_List *list, int d_node_time, int bez_time_div) 
{
    for (int i = BEZ_CHANGE_FREQ; i > BEZ_DIV_LIM; --i) {
        if (list->head == list->tail)
            break;
        if (d_node_time > i*(bez_time_div/2))
            break;
        if (d_node_time < (i-1)*(bez_time_div/2)) // if less than previous division, skip
            continue;
        list->grow_ctr_points[i - 1 - BEZ_DIV_LIM].x = list->tail->point.x;
        list->grow_ctr_points[i - 1 - BEZ_DIV_LIM].y = list->tail->point.y;
    }
}

void set_grow_bez_point (Stem_List *list, int d_node_time, int bez_time_div) 
{
    for (int i = BEZ_CHANGE_FREQ; i > BEZ_DIV_LIM; --i) {
        if (list->head == list->tail)
            break;
        if (d_node_time > i*bez_time_div) // if more than current division, done
            break;
        if (d_node_time < (i-1)*bez_time_div) // if less than previous division, skip
            continue;
        
        Stem_Node *pre_tail = list->head;
        while (pre_tail != NULL && pre_tail->next != list->tail) {
            pre_tail = pre_tail->next;
        }

        double time_ratio = (double)(i*(bez_time_div/2)) / (double)d_node_time;

        D_Point new_bez_point = bez_ctr_from_curve(pre_tail->point.x, pre_tail->point.y,
                                                   list->grow_ctr_points[i - 1 - BEZ_DIV_LIM].x, 
                                                   list->grow_ctr_points[i - 1 - BEZ_DIV_LIM].y,
                                                   list->tail->point.x, list->tail->point.y, time_ratio);
        list->tail->bez_ctr_point.x = new_bez_point.x;
        list->tail->bez_ctr_point.y = new_bez_point.y;
        list->tail->bez_ctr_time = gTime;
    }
}

void update_growing_stem_bez_point (Stem_List *list) 
{
    if (list->length < list->length_lim) { // if still growing
        int d_node_time = gTime - list->tail->creation_time; // time the node has existed for
        // fraction of time over which there is a constant point to determine the Bezier control point
        int bez_time_div = list->growth_period/BEZ_CHANGE_FREQ;

        // determine stored growth point position values for Bezier control point in stem
        store_grow_position(list, d_node_time, bez_time_div);

        // use stored values to calculate Bezier control point for growth point
        set_grow_bez_point(list, d_node_time, bez_time_div);
    }
}

void set_flower_size (Stem_List *list)
{
    if (list->length < list->length_lim) { return; } // if still growing, no flower
    if (speed == 0) { return; } // program paused, stop growing

    if (list->max_flower_size == INVALID_VAL)
        list->max_flower_size = average_surround(list->tail->point.x, list->tail->point.y, FLOWER_SIZE_RANGE);
    else if (list->curr_flower_size != list->max_flower_size)
        list->curr_flower_size = list->curr_flower_size + (rand() % 2);
}

void update_flower_size (Stem_List *list)
{
    list->old_flower_size = list->curr_flower_size;
}

void branch_stem (Stem_List *list) 
{
    Stem_Node *n = list->head;
    if (n != NULL)
        n = n->next; // do not branch from head

	while (n != NULL && n->next != NULL) { // do not branch from tail
        if (n->branchable == TRUE && n->branching_stem == NULL) { // if no branching stem but can branch
            if (gTime - n->next->creation_time >= n->branch_period) {
                Stem_List * new_list = (Stem_List*) malloc(sizeof(Stem_List));
                if (new_list == NULL) // check if enough memory to allocate
                    return;
                short int flower_color = randomize_color_change(list->flower_color, BRANCH_COLOR_CHANGE, BRANCH_COLOR_CHANGE/2, BRANCH_COLOR_CHANGE);
                init_stem_list(new_list, n->point.x, n->point.y, list->color, flower_color, list->flower_center_color, darkify_color(flower_color), list->num_branches + 1);
                add_stem_node(new_list, n->point.x, n->point.y, INVALID_VAL, INVALID_VAL, n->point.reverse_x, n->point.reverse_y);
                
                n->branching_stem = new_list;
            }
        }

        n = n->next;
    }
}

void draw_branching_stems_rec (Stem_List *list)
{
    Stem_Node *n = list->head;
    if (n != NULL)
        n = n->next; // do not branch from head
	while (n != NULL && n->next != NULL) {
        if (n->branchable == TRUE && n->branching_stem != NULL) {
            draw_stem (n->branching_stem);
        }

        n = n->next;
    }
}

void draw_stem (Stem_List *list)
{
    add_new_stem_node(list); // add new nodes
    branch_stem(list); // add new stems branching off current stem
    draw_old_stem_nodes(list, BLACK); // clear stem drawing
    draw_old_stem_flower(list); // clear flower drawing
    update_old_stem_points(list); // update old points
    move_stem_points(list); // move points of stem
    update_growing_stem_bez_point(list); // set new Bezier curve control point for growing point
    update_flower_size (list); // update old flower size
    set_flower_size (list); // set flower size of stem based on room available
    draw_stem_nodes(list, list->color); /// draw stem
    draw_stem_flower(list); // draw flower
    draw_branching_stems_rec(list); // go through list and draw other stems in list, recursively
}


/* Draw flower */
void draw_flower (int x, int y, short int petal_color, short int center_color, short int border_color, short int size) {
	int radius = 3;

    // draw the petals
    if (size >= 5) { // fifth layer of petals
        plot_ellipse(x + (radius*1.5), y + (radius*3.5), radius*1.75, radius*1.75, petal_color, border_color);
        plot_ellipse(x - (radius*1.5), y + (radius*3.5), radius*1.75, radius*1.75, petal_color, border_color);
        plot_ellipse(x + (radius*1.5), y - (radius*3.5), radius*1.75, radius*1.75, petal_color, border_color);
        plot_ellipse(x - (radius*1.5), y - (radius*3.5), radius*1.75, radius*1.75, petal_color, border_color);

        plot_ellipse(x + (radius*3.5), y + (radius*1.5), radius*1.75, radius*1.75, petal_color, border_color);
        plot_ellipse(x - (radius*3.5), y + (radius*1.5), radius*1.75, radius*1.75, petal_color, border_color);
        plot_ellipse(x + (radius*3.5), y - (radius*1.5), radius*1.75, radius*1.75, petal_color, border_color);
        plot_ellipse(x - (radius*3.5), y - (radius*1.5), radius*1.75, radius*1.75, petal_color, border_color);
    }

    if (size >= 4) { // fourth layer of petals
        plot_ellipse(x + (3*radius), y, radius*1.5, radius*1.5, petal_color, border_color);
        plot_ellipse(x - (3*radius), y, radius*1.5, radius*1.5, petal_color, border_color);
        plot_ellipse(x, y + (3*radius), radius*1.5, radius*1.5, petal_color, border_color);
        plot_ellipse(x, y - (3*radius), radius*1.5, radius*1.5, petal_color, border_color);

        plot_ellipse(x + (2*radius), y + (2*radius), radius*1.5, radius*1.5, petal_color, border_color);
        plot_ellipse(x - (2*radius), y + (2*radius), radius*1.5, radius*1.5, petal_color, border_color);
        plot_ellipse(x + (2*radius), y - (2*radius), radius*1.5, radius*1.5, petal_color, border_color);
        plot_ellipse(x - (2*radius), y - (2*radius), radius*1.5, radius*1.5, petal_color, border_color);
    }

    if (size >= 3) { // third layer of petals
        plot_ellipse(x + (radius*0.75), y + (2*radius), radius, radius, petal_color, border_color);
        plot_ellipse(x - (radius*0.75), y + (2*radius), radius, radius, petal_color, border_color);
        plot_ellipse(x + (radius*0.75), y - (2*radius), radius, radius, petal_color, border_color);
        plot_ellipse(x - (radius*0.75), y - (2*radius), radius, radius, petal_color, border_color);

        plot_ellipse(x + 2 * radius, y + (radius*0.75), radius, radius, petal_color, border_color);
        plot_ellipse(x - 2 * radius, y + (radius*0.75), radius, radius, petal_color, border_color);
        plot_ellipse(x + 2 * radius, y - (radius*0.75), radius, radius, petal_color, border_color);
        plot_ellipse(x - 2 * radius, y - (radius*0.75), radius, radius, petal_color, border_color);
    }

    if (size >= 2) { // second layer of petals
        plot_ellipse(x + 2*(radius*0.75), y, radius, radius, petal_color, border_color);
        plot_ellipse(x - 2*(radius*0.75), y, radius, radius, petal_color, border_color);
        plot_ellipse(x, y + 2*(radius*0.75), radius, radius, petal_color, border_color);
        plot_ellipse(x, y - 2*(radius*0.75), radius, radius, petal_color, border_color);
    }

    if (size >= 1) { // first layer of petals
        plot_ellipse(x + radius, y + radius, radius, radius, petal_color, border_color);
        plot_ellipse(x - radius, y + radius, radius, radius, petal_color, border_color);
        plot_ellipse(x + radius, y - radius, radius, radius, petal_color, border_color);
        plot_ellipse(x - radius, y - radius, radius, radius, petal_color, border_color);
    }

    // draw center of the flower
    plot_ellipse(x, y, radius*0.75, radius*0.75, center_color, border_color);
}

/* Draw simple shapes and lines */
void plot_pixel(int x, int y, short int line_color)
{
    if (x < 0 || y < 0 || x > RESOLUTION_X || y > RESOLUTION_Y)
        return;
    *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = line_color;
}

short int get_pixel (int x, int y)
{
    if (x < 0 || y < 0 || x > RESOLUTION_X || y > RESOLUTION_Y)
        return BLACK;
    volatile short int pixel = *(short int *)(FPGA_ONCHIP_BASE + (y << 10) + (x << 1));
    pixel = pixel & 0x0000FFFF;
    return pixel;
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
void plot_ellipse(int x0, int y0, int r1, int r2, short int color, short int border_color) 
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
        plot_pixel(x0 - abs(x), y0 + y, border_color); // Quadrant 1
        plot_pixel(x0 + abs(x), y0 + y, border_color); // Quadrant 2
        plot_pixel(x0 + abs(x), y0 - y, border_color); // Quadrant 3
        plot_pixel(x0 - abs(x), y0 - y, border_color); // Quadrant 4

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

void clear_screen ()
{
    for (int x = 0; x < RESOLUTION_X; ++x) {
        for (int y = 0; y < RESOLUTION_Y; ++y) {
            *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = BLACK;
        }
    }
    
}

void clear_all ()
{
    volatile int * pixel_ctrl_ptr = (int *)PIXEL_BUF_CTRL_BASE;
    free_plant_list(&plants);
    wait_for_vsync(); // swap front and back buffers on VGA vertical sync
    pixel_buffer_start = *(pixel_ctrl_ptr + 1);
    clear_screen();
    wait_for_vsync(); // swap front and back buffers on VGA vertical sync
    pixel_buffer_start = *(pixel_ctrl_ptr + 1);
    clear_screen();
}


/* Colour changing functions */
// returns a random colour based on parameters limiting the amount of red, green, and blue, 
// the minimum amount for each is 0, the maximum red and blue input is 31, maximum green input is 63
short int randomize_color (short int min_red, short int max_red, short int min_green, short int max_green, short int min_blue, short int max_blue)
{
    // ensure the values remain within bounds
    min_red = (min_red < MIN_RGB) ? MIN_RGB : min_red;
    max_red = (max_red > MAX_RED_BLUE) ? MAX_RED_BLUE : max_red;
    min_green = (min_green < MIN_RGB) ? MIN_RGB : min_green;
    max_green = (max_green > MAX_GREEN) ? MAX_GREEN : max_green;
    min_blue = (min_blue < MIN_RGB) ? MIN_RGB : min_blue;
    max_blue = (max_blue > MAX_RED_BLUE) ? MAX_RED_BLUE : max_blue;

    // randomize the individual rgb bits
    short int red = rand() % (max_red - min_red + 1) + min_red; // bits 15 - 11
    short int green = rand() % (max_green - min_green + 1) + min_green; // bits 10 - 5
    short int blue = rand() % (max_blue - min_blue + 1) + min_blue; // bits 4 - 0

    // combine rgb bits into single colour
    short int color = (red << 11) + (green << 5) + blue;
    return color;
}

short int randomize_color_change (short int color, short int red_change, short int green_change, short int blue_change)
{
    short int red = (color & 0b1111100000000000) >> 11;
    short int green = (color & 0b0000011111100000) >> 5;
    short int blue = (color & 0b0000000000011111);

    short int min_red = red - red_change;
    short int max_red = red + red_change;
    short int min_green = green - green_change;
    short int max_green = green + green_change;
    short int min_blue = blue - blue_change;
    short int max_blue = blue + blue_change;

    short int new_color = randomize_color(min_red, max_red, min_green, max_green, min_blue, max_blue);
    return new_color;
}

short int darkify_color (short int color)
{
    short int red = (color & 0b1111100000000000) >> 11;
    short int green = (color & 0b0000011111100000) >> 5;
    short int blue = (color & 0b0000000000011111);

    red /= DARK_COLOR_CHANGE;
    green /= DARK_COLOR_CHANGE;
    blue /= DARK_COLOR_CHANGE;

    short int dark_color = (red << 11) + (green << 5) + blue;
    return dark_color;
}

short int randomize_flower_color () 
{
    short int min_red, max_red, min_blue, max_blue, min_green, max_green;

    int random = rand() % 4;

    if (random == 0) { // red
        min_red = 7 * MAX_RED_BLUE / 8;
        max_red = MAX_RED_BLUE;
        min_green = MAX_GREEN / 7;
        max_green = 2 * MAX_GREEN / 5;
        min_blue = MAX_RED_BLUE / 8;
        max_blue = MAX_RED_BLUE / 3;
    }
    else if (random == 1) { // yellow
        min_red = 5 * MAX_RED_BLUE / 6;
        max_red = MAX_RED_BLUE;
        min_green = 4 * MAX_GREEN / 6;
        max_green = 5 * MAX_GREEN / 6;
        min_blue = MAX_RED_BLUE / 6;
        max_blue = MAX_RED_BLUE / 4;
    }
    else if (random == 2) { 
        if (rand() % 2 == 0) { // blue
            min_red = MAX_RED_BLUE / 6;
            max_red = MAX_RED_BLUE / 4;
            min_blue = 7 * MAX_RED_BLUE / 8;
            max_blue = MAX_RED_BLUE;
            min_green = MAX_GREEN / 6;
            max_green = 2 * MAX_GREEN / 5;
        }
        else { // purple
            min_red = 5 * MAX_RED_BLUE / 6;
            max_red = MAX_RED_BLUE;
            min_blue = 5 * MAX_RED_BLUE / 6;
            max_blue = MAX_RED_BLUE;
            min_green = MAX_GREEN / 6;
            max_green = 2 * MAX_GREEN / 5;
        }
    }
    else { // white
        min_red = 7 * MAX_RED_BLUE / 8;
        max_red = MAX_RED_BLUE;
        min_blue = 7 * MAX_RED_BLUE / 8;
        max_blue = MAX_RED_BLUE;
        min_green = 7 * MAX_GREEN / 8;
        max_green = MAX_GREEN;
    }

    randomize_color(min_red, max_red, min_green, max_green, min_blue, max_blue);
}

void change_plant_colors (Plant_List *list)
{
    Plant_Node *plant = list->head; // go through all plants
    while (plant != NULL) {
        change_stem_colors(plant->stem, BLACK, BLACK, BLACK);
        plant = plant->next;
    }
}

void change_stem_colors (Stem_List *list, short int color, short int flower_color, short int flower_center_color)
{
    if (list->num_branches == 0) { // no branches, new colours
        color = randomize_color(STEM_MIN_RED, STEM_MAX_RED, STEM_MIN_GREEN, STEM_MAX_GREEN, STEM_MIN_BLUE, STEM_MAX_BLUE);
        flower_color = randomize_flower_color();
        flower_center_color = (rand() % 2 == 0) ? DARK_CENTER : YELLOW_CENTER;
    }
    else {
        flower_color = randomize_color_change(flower_color, BRANCH_COLOR_CHANGE, BRANCH_COLOR_CHANGE/2, BRANCH_COLOR_CHANGE);
    }
    short int border_color = darkify_color(flower_color);

    list->color = color;
    list->flower_color = flower_color;
    list->flower_center_color = flower_center_color;
    list->border_color = border_color;

    Stem_Node *node = list->head;
    while (node != NULL) {
        if (node->branching_stem != NULL)
            change_stem_colors(node->branching_stem, color, flower_color, flower_center_color);
        node = node->next;
    }
}


/* Helper functions */
void find_surround (int x, int y, int range, int *left_dist, int *right_dist, int *up_dist, int *down_dist) 
{
    // closest point left, right, up, and down of the given point
    *left_dist = INVALID_VAL;
    *right_dist = INVALID_VAL;
    *up_dist = INVALID_VAL;
    *down_dist = INVALID_VAL;

    for (int i = 1; i < range; ++i) {
        if ((*left_dist == INVALID_VAL) && (get_pixel(x - i, y) != BLACK || x - i < 0))
            *left_dist = i;
        if ((*right_dist == INVALID_VAL) && (get_pixel(x + i, y) != BLACK || x + i > RESOLUTION_X))
            *right_dist = i;
        if ((*up_dist == INVALID_VAL) && (get_pixel(x, y - i) != BLACK || x - i < 0))
            *up_dist = i;
        if ((*down_dist == INVALID_VAL) && (get_pixel(x, y + i) != BLACK || x + i > RESOLUTION_Y))
            *down_dist = i;
    }
}

double average_surround (int x, int y, int range) 
{
    // closest point left, right, up, and down of the given point
    int left_dist, right_dist, up_dist, down_dist;
    find_surround(x, y, range, &left_dist, &right_dist, &up_dist, &down_dist);

    if (left_dist == INVALID_VAL) 
        left_dist = range;
    if (right_dist == INVALID_VAL) 
        right_dist = range;
    if (up_dist == INVALID_VAL) 
        up_dist = range;
    if (down_dist == INVALID_VAL) 
        down_dist = range;

    return (double)(left_dist + right_dist + up_dist + down_dist) / 3.0;
}

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

D_Point vector_field (double x, double y, double time) {
    D_Point point;
    //point.x = 0.5 * sin((x + y)/FIELD_SCALE) - cos(time/(FIELD_SCALE));
    //point.y = - 0.5 * cos((x + y)/FIELD_SCALE) + sin(time/(FIELD_SCALE));
    point.x = sin((sin(time/FIELD_SCALE/4)*x + sin(time/FIELD_SCALE)*y)/FIELD_SCALE);
    point.y = -cos((cos(time/FIELD_SCALE)*x + cos(time/FIELD_SCALE/4)*y)/FIELD_SCALE);
    return point;
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


/* Modify program from interrupt flags */
void pause_program ()
{
    if (pause_program_flag == TRUE) {
        pause_program_flag = FALSE;
        if (speed == 0) {
            speed = old_speed;
            speed_scale = old_speed_scale;
        }
        else {
            old_speed = speed;
            old_speed_scale = speed_scale;
            speed = 0;
            speed_scale = 0;
        }
    }
}

void clear_program ()
{
    if (clear_all_flag == TRUE) {
        clear_all_flag = FALSE;
        clear_all();
        init_start_values();
    }
}

void change_program_speed ()
{
    if (change_speed_flag == TRUE) {
        change_speed_flag = FALSE;
        // change the speed
        if (speed == 1) {
            speed = 2;
            speed_scale = DEFAULT_SPEED * 2;
        }
        else if (speed == 2) {
            speed = 3;
            speed_scale = DEFAULT_SPEED * 3;
        }
        else if (speed == 3) {
            speed = 1;
            speed_scale = DEFAULT_SPEED;
        }
    }
}

void mouse_clicked ()
{
    if (mouse_clicked_flag == TRUE) {
        mouse_clicked_flag = FALSE;
        add_plant_node(&plants, mouse_x, mouse_y);
    }
}

void draw_mouse ()
{
    if (old_mouse_x != 0 && old_mouse_y != 0)
        plot_ellipse(old_mouse_x, old_mouse_y, 2, 2, BLACK, BLACK); // erase old mouse
        
    old_mouse_x = mouse_x;
    old_mouse_y = mouse_y;
    mouse_x = PS2_mouse_x;
    mouse_y = PS2_mouse_y;

    if (mouse_x != 0 && mouse_y != 0)
        plot_ellipse(mouse_x, mouse_y, 2, 2, WHITE, BLACK); // draw new mouse
}


/* Interrupt service routines */
void interval_timer_ISR(void)
{
    // interal timer base address
    volatile int * interval_timer_ptr = (int *)TIMER_BASE;
    *(interval_timer_ptr) = 0; // clear the interrupt
    
    gTime += speed;
}

void pushbutton_ISR(void) 
{
    // KEY base address
    volatile int * KEY_ptr = (int *) KEY_BASE;
    
    int press = *(KEY_ptr + 3); // read the pushbutton interrupt register
    *(KEY_ptr + 3) = press; // clear the interrupt

    if (press & 0x1) { // KEY0
        // reset the animation
        clear_all_flag = TRUE;
    }
    else if (press & 0x2) { // KEY1
        // pause or un-pause the program
        pause_program_flag = TRUE;
    }
    else if (press & 0x4) { // KEY2
        change_speed_flag = TRUE;
    }
    else { // press & 0x8, which is KEY3
        change_plant_colors(&plants); // change the colours of the flower and stem
    }
}

void PS2_ISR(void)
{
    // PS2 base address
    volatile int * PS2_ptr = (int *) PS2_BASE;

    *(PS2_ptr + 1) = 0x1; // clear the interrupt

    int PS2_data, RVALID;
    unsigned char byte1 = 0;
    unsigned char byte2 = 0; // X movement
    unsigned char byte3 = 0; // Y movement

    while (1) {
        PS2_data = *(PS2_ptr); // read the Data register in the PS/2 port
        RVALID = PS2_data & 0x8000; // extract the RVALID field

        if (RVALID) {
            byte1 = byte2;
            byte2 = byte3;
            byte3 = PS2_data & 0xFF;
			printf ("%d\n", byte3);
        }
        else {
            break;
        }
    }

    // adjust for two's complement in the X and Y movement
    if (byte2 & 0x80) // if 8th bit is 1, then is negative
        byte2 = abs(256 - byte2);
    if (byte3 & 0x80) // if 8th bit is 1, then is negative
        byte3 = abs(256 - byte3);

	// add or subtract to current x position
	if (byte1 & 0x10) // bit 4 (X sign bit)
		PS2_mouse_x -= byte2;
	else
		PS2_mouse_x += byte2;

	// add or subtract to current y position
	if (byte1 & 0x20)
		PS2_mouse_y -= byte3;
	else
		PS2_mouse_y += byte3;
	
	// check if mouse went out of bounds
	if (PS2_mouse_x < 0)
		PS2_mouse_x = 0;
	else if (PS2_mouse_x > RESOLUTION_X)
		PS2_mouse_x = RESOLUTION_X;
	
	if (PS2_mouse_y < 0)
		PS2_mouse_y = 0;
	else if (PS2_mouse_y > RESOLUTION_Y)
		PS2_mouse_y = RESOLUTION_Y;
    
    // detect if mouse was clicked
	if (byte1 & 0x1) {
		mouse_clicked_flag = TRUE;
	}
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

    /* Configure the FPGA interval timer and KEYs interrupts */
    *((int *)0xFFFED848) = 0x00000101;
    *((int *)0xFFFED108) = 0x00000300;

    /* Configure the PS2 interrupt */
    config_interrupt (79, 1);

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

/*
 * Configure Set Enable Registers (ICDISERn) and Interrupt Processor Target Registers (ICDIPTRn).
 * The default (reset) values are used for other registers in the GIC.
 * From the Using the ARM Generic Interrupt Controller Manual
 */
void config_interrupt(int N, int CPU_target)
{
    int reg_offset, index, value, address;

    /* Configure the Interrupt Set-Enable Registers (ICDISERn).
     * reg_offset = (integer_div(N / 32) * 4
     * value = 1 << (N mod 32) */
    reg_offset = (N >> 3) & 0xFFFFFFFC;
    index = N & 0x1F;
    value = 0x1 << index;
    address = 0xFFFED100 + reg_offset;

    /* Now that we know the register address and value, set the appropriate bit */
    *(int *)address |= value;

    /* Configure the Interrupt Processor Targets Register (ICDIPTRn)
     * reg_offset = integer_div(N / 4) * 4
     * index = N mod 4 */
    reg_offset = (N & 0xFFFFFFFC);
    index = N & 0x3;
    address = 0xFFFED800 + reg_offset + index;

    /* Now that we know the register address and value, write to (only) the appropriate byte */
    *(char *)address = (char)CPU_target;
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

// Setup the PS2 for mouse input
void config_PS2(void)
{
    volatile int * PS2_ptr = (int *)PS2_BASE; // PS2 address
    *(PS2_ptr) = 0xFF; // reset
    *(PS2_ptr + 1) = 0x1; // enable interrupts from the PS2 port
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
    else if (interrupt_ID == PS2_IRQ)
        PS2_ISR();
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
