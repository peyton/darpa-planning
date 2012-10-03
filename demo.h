/*******************************************************************/
/*
demo.h: demo astar
*/
/*******************************************************************/

/******* COMPILER DIRECTIVES ****************************************/

/******* INCLUDES ***************************************************/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef WIN32
#define M_PI 3.14159f
#endif

/******* DEFINES ***************************************************/

// These are 0 or 1
#define DO_PRINTF 1
#define QUIT_ON_FIRST_SUCCESS 0

// number of numbers in (X, Y)
#define N_XY 2

// components of state
#define XX 0
#define YY 1
#define ANGLE 2
#define SIDE 3
#define N_X 4

// sides
#define LEFT 0
#define RIGHT 1

// resolution of terrain map, X and Y distance/pixel should be equal
#define TERRAIN_N_X 500
#define TERRAIN_N_Y 500

// constants
#define BIG_COST 1e10f  // big value for cost map
#define BIG_VALUE 1e30f // big value for heuristic function

// SHOULD BE VARIABLES
#define INFLATION 2.0f // heuristic inflation factor // 1.2 good
#define STEP_COST_WEIGHT 10.0f // scale factor on cost map
#define STEP_LENGTH 0.55f // meters
#define HIP_WIDTH 0.25f // meters
#define CLOSE_ENOUGH ((float) STEP_LENGTH/2) // close enough to goal to quit?
#define DUPLICATE_DISTANCE_THRESHOLD 0.1f // duplicate detection threshold
#define CUTOFF_INCREMENT 5*STEP_LENGTH // cutoff if priority too high

// FIX THIS
#define DISTANCE 0.05f // should be WIDTH/T_MAX_N_X or HEIGHT/T_MAX_N_Y

#define UINT unsigned int
/******* TYPEDEFS *****************************************************/

typedef struct terrain
{
  int resolution[N_XY]; // x and y resolution of cost map
  int n_cells; // number of cells in cost map
  float min[N_XY]; // borders of cost map in task coordinates
  float max[N_XY];
  float current[N_X]; // current state
  float goal[N_X]; // goal state: goal for COM
  float *true_cost_map; // complete true cost map
  float *perceived_cost_map; // incomplete perceived cost map
  float *known_terrain; // what is known terrain

  float inc; // one increment is distance/pixel
  // should be equal in x and y
  int perception_radius; // how far can we see in pixels?
}
  TERRAIN;

/*****************************************************/

typedef struct astar_node
{
  int id; // unique node id for debugging

  int depth; // depth in A* tree

  float state[N_X]; // actually foot location
  float com[N_X]; // actually COM location

  // allow location map to be different from terrain map
  int terrain_index; // location of node in terrain cost map
  int location_index; // location of node in location/duplicate map

  // parent, children, and siblings in A* tree
  struct astar_node *parent;
  struct astar_node *children;
  struct astar_node *sibling;

  // next element in any q or list (priority_q, done_list)
  struct astar_node *next; 

  // next element in location list
  struct astar_node *location_next; 

  float cost_from_start;
  float one_step_cost;
  float terrain_cost;
  float cost_to_go;
  float priority;
}
  ASTAR_NODE;

/*****************************************************/

typedef struct astar
{
  int active; // is astar in use or "free"
  int done; // are we done searching?
  ASTAR_NODE *priority_q;
  ASTAR_NODE *done_list;
  ASTAR_NODE *free_list;
  ASTAR_NODE *at_goal; // nodes that are at goal
  ASTAR_NODE *root_node; // keep track of root node
  ASTAR_NODE **location_map; // keep track of where nodes are
  TERRAIN *tt;

  // search control
  int policy;
  float best_value;

  // for debugging
  int expand_next_node_count;
}
  ASTAR;

/*****************************************************/

typedef struct path_node
{
  int id; // unique node id for debugging

  int an_id; // node id of ASTAR_NODE we copied from, for debugging

  float state[N_X]; // actually foot location
  float com[N_X]; // actually COM location

  struct path_node *previous; 
  struct path_node *next; 

  float cost_from_start;
  float one_step_cost;
  float terrain_cost;
  float cost_to_go;
  float priority;
}
  PATH_NODE;

/******* DEFINITIONS *****************************************************/

void add_an_to_path( ASTAR *aa, ASTAR_NODE *an );
void displaya();

/******* GLOBALS *****************************************************/

// Seems OpenGL requires us to have globals
extern TERRAIN *t1; // terrain map
extern ASTAR *a1; // A* search tree
extern PATH_NODE *p1; // actual path taken
extern int path_done; // flag: is goal reached by robot?
extern int wa; // Window for ASTAR stuff

// debugging flags
extern int debug_a;
extern int debug_plan;
extern int debug_free;

// for debugging
extern int astar_node_count; // needs to be global since free list shared across
// all astar searches.
