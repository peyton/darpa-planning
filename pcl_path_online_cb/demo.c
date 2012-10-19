/*******************************************************************/
/*
demo.c: demo astar
*/
/*******************************************************************/

//#import "demo.h"
#include "demo.h"

/******* GLOBALS *****************************************************/

// Seems OpenGL requires us to have globals
TERRAIN *t1 = NULL; // terrain map
ASTAR *a1 = NULL; // A* search tree
PATH_NODE *p1 = NULL; // actual path taken
int path_done = 0; // flag: is goal reached by robot?
int wa = -1; // Window for ASTAR stuff

// debugging flags
int debug_a = 0;
int debug_plan = 0;
int debug_free = 0;

// for debugging
int astar_node_count = 0; // needs to be global since free list shared across
// all astar searches.

/******* TERRAIN ****************************************************/

TERRAIN *create_terrain()
{
  int i;
  TERRAIN *tt;

  tt = (TERRAIN *) malloc( sizeof( TERRAIN ) );
  if ( tt == NULL )
    {
      fprintf( stderr, "Can't allocate terrain.\n" );
      exit( -1 );
    }

  tt->resolution[XX] = TERRAIN_N_X;
  tt->resolution[YY] = TERRAIN_N_Y;
  tt->n_cells = tt->resolution[XX]*tt->resolution[YY];

  tt->true_cost_map = (float *) malloc( tt->n_cells*sizeof( float ) );
  if ( tt->true_cost_map == NULL )
    {
      fprintf( stderr, "Can't allocate true terrain cost map.\n" );
      exit( -1 );
    }

  tt->perceived_cost_map = (float *) malloc( tt->n_cells*sizeof( float ) );
  if ( tt->perceived_cost_map == NULL )
    {
      fprintf( stderr, "Can't allocate perceived terrain cost map.\n" );
      exit( -1 );
    }

  tt->known_terrain = (float *) malloc( tt->n_cells*sizeof( float ) );
  if ( tt->known_terrain == NULL )
    {
      fprintf( stderr, "Can't allocate known terrain cost map.\n" );
      exit( -1 );
    }

  for ( i = 0; i < tt->n_cells; i++ )
    {
      tt->true_cost_map[i] = 0;
      tt->perceived_cost_map[i] = 0;
      tt->known_terrain[i] = 0;
    }

  return tt;
}

/*******************************************************************/

void terrain_indices( TERRAIN *tt, float x, float y,
		      int *ix, int *iy, int *index )
{
  int iix, iiy;

  if ( x < tt->min[XX]
       || x > tt->max[XX]
       || y < tt->min[YY]
       || y > tt->max[YY] )
    {
      fprintf( stderr,
	       "terrain_indices x, y out of bounds: %g: %g %g; %g: %g %g\n",
	       x, tt->min[XX], tt->max[XX],
	       y, tt->min[YY], tt->max[YY] );
      exit( -1 );
    }

  if ( x == tt->max[XX] )
    iix = tt->resolution[XX] - 1;
  else
    iix = (int) (tt->resolution[XX]*(x - tt->min[XX])
		 /(tt->max[XX] - tt->min[XX]));

  if ( y == tt->max[YY] )
    iiy = tt->resolution[YY] - 1;
  else
    iiy = (int) (tt->resolution[YY]*(y - tt->min[YY])
		 /(tt->max[YY] - tt->min[YY]));

  if ( ix != NULL )
    *ix = iix;

  if ( iy != NULL )
    *iy = iiy;

  if ( index != NULL )
    *index = iix*tt->resolution[YY] + iiy;
}

/*******************************************************************/

float get_perceived_cost_map_pixel( TERRAIN *tt, int ix, int iy )
{
  int index;

  if ( ix < 0 || ix >= tt->resolution[XX] )
    return BIG_COST;

  if ( iy < 0 || iy >= tt->resolution[YY] )
    return BIG_COST;

  index = ix*tt->resolution[YY] + iy;

  return tt->perceived_cost_map[ index ];
}

/*******************************************************************/

float get_true_cost_map_pixel( TERRAIN *tt, int ix, int iy )
{
  int index;

  if ( ix < 0 || ix >= tt->resolution[XX] )
    return BIG_COST;

  if ( iy < 0 || iy >= tt->resolution[YY] )
    return BIG_COST;

  index = ix*tt->resolution[YY] + iy;

  return tt->true_cost_map[ index ];
}

/*******************************************************************/

void set_perceived_cost_map_pixel( TERRAIN *tt, int ix, int iy, float value )
{
  int index;

  if ( ix < 0 || ix >= tt->resolution[XX] )
    return;

  if ( iy < 0 || iy >= tt->resolution[YY] )
    return;

  index = ix*tt->resolution[YY] + iy;

  tt->perceived_cost_map[ index ] = value;
  tt->known_terrain[ index ] += 1.0;
}

/*******************************************************************/

void set_true_cost_map_pixel( TERRAIN *tt, int ix, int iy, float value )
{
  int index;

  if ( ix < 0 || ix >= tt->resolution[XX] )
    return;

  if ( iy < 0 || iy >= tt->resolution[YY] )
    return;

  index = ix*tt->resolution[YY] + iy;

  tt->true_cost_map[ index ] = value;
}

/*******************************************************************/

void generate_true_cost_map( TERRAIN *tt, float terrain_value[TERRAIN_N_X][TERRAIN_N_Y] )
{
  int i, j;
  int ix, ixc;
  int iy, iyc;
  int ixc2, iyc2;
  int kx, ky;
  float value[TERRAIN_N_X][TERRAIN_N_Y];
  float filter_coeff = 0.6f;

  tt->inc = (tt->max[XX] - tt->min[XX])/TERRAIN_N_X;
  if ( fabsf( tt->inc - (tt->max[YY] - tt->min[YY])/TERRAIN_N_Y ) > 1e-3 )
    {
      // Need to implement more sophisticated tt->inc 
      fprintf( stderr, "X and Y distance/pixel not equal: %g %g\n",
	       tt->inc, (tt->max[YY] - tt->min[YY])/TERRAIN_N_Y );
      exit( -1 );
    }

  // See how far away perception horizon is.
  terrain_indices( tt, 0.0f, 0.0f, &ixc, &iyc, NULL );
  terrain_indices( tt, 6.0f, 6.0f, &ixc2, &iyc2, NULL );
  tt->perception_radius = ixc2 - ixc;

  // current and goal states
  tt->current[XX] = 15;
  tt->current[YY] = 10;
  tt->current[ANGLE] = (float) (M_PI/2);
  tt->current[SIDE] = LEFT;

  tt->goal[XX] = 15;
  tt->goal[YY] = 15;
  tt->goal[ANGLE] = tt->current[ANGLE];
  // should ignore this in calculating distance to goal
  tt->current[SIDE] = LEFT; 

  // initialize cost map
  for ( i = 0; i < tt->resolution[XX]; i++ )
    for ( j = 0; j < tt->resolution[YY]; j++ )
      set_true_cost_map_pixel( tt, i, j, terrain_value[i][j] );
         	
  // BLUR COST MAP
  // blur each column
  for ( ix = 0; ix < tt->resolution[XX]; ix++ )
    { 
      for ( iy = 0; iy < tt->resolution[YY]; iy++ )
		{
			value[ix][iy] = 0;
            for (ky = -2; ky <= 2; ky++)
                for (kx = -2; kx <= 2; kx++)
                    value[ix][iy] += get_true_cost_map_pixel(tt,ix + kx, iy + ky);
			value[ix][iy] = value[ix][iy]/25;
			
		}
    }
  for ( ix = 0; ix < tt->resolution[XX]; ix++ )
	  for ( iy = 0; iy < tt->resolution[YY]; iy++ )
		set_true_cost_map_pixel( tt, ix, iy, value[ix][iy]);
	
}

/******* A* SEARCH ****************************************************/
/*******************************************************************/
// used in generate_child(), heuristic()
// should scale by dimension (x,y vs. angle)
// ignore side

float xya_distance( ASTAR *aa, float *s1, float *s2 )
{
  int i, j;
  float diff;
  float score = 0;

  // X. Y distance
  for ( i = XX; i <= YY; i++ )
    {
      diff = s1[i] - s2[i];
      score += diff*diff;
    }

  // ANGLE
  i = ANGLE;
  diff = s1[i] - s2[i];
  // get -pi <= diff <= pi
  for ( j = 0; ; j++ )
    {

      if ( j > 5 )
		{
		printf( "xya_distance: %d %20.15f %20.15f %20.15f %20.15f\n", j,
		  diff, M_PI, diff - ((float) (2*M_PI)),
		  diff + ((float) (2*M_PI)) );
		}
      if ( j > 10 )
		{
		fprintf( stderr, "xya_distance broken.\n" );
		exit( -1 );
		}

      if ( diff > M_PI )
		{
		diff -= (float) (2*M_PI - 1e-6);
		continue;
		}
      if ( diff < -M_PI )
		{
		diff += (float) (2*M_PI + 1e-6);
		continue;
		}
      break;
    }
  score += 0.5*diff*diff;

  return sqrtf( score );
}

/*******************************************************************/
// used in handle_duplicates
// should scale by dimension (x,y vs. angle)

float state_distance( ASTAR *aa, float *s1, float *s2 )
{
  int i;
  float diff;
  float score = 0;

  score = xya_distance( aa, s1, s2 );

  // side distance
  i = SIDE;
  diff = s1[i] - s2[i];

  // side is not a meaningful Euclidean distance anyway
  return score + diff*diff;
}

/*******************************************************************/

ASTAR_NODE* create_astar_node( ASTAR *aa )
{
  ASTAR_NODE *an;

  if ( aa->free_list != NULL )
    {
      an = aa->free_list;
      aa->free_list = an->next;
    }
  else
    {
      an = (ASTAR_NODE *) malloc( sizeof( ASTAR_NODE ) );
      an->id = astar_node_count++;
    }

  if ( an == NULL )
    {
      fprintf( stderr, "Can't allocate astar node.\n" );
      exit( -1 );
    }
  an->depth = -1;
  an->state[XX] = -1e10;
  an->state[YY] = -1e10;
  an->state[ANGLE] = -1e10;
  an->state[SIDE] = -1;
  an->com[XX] = -1e10;
  an->com[YY] = -1e10;
  an->com[ANGLE] = -1e10;
  an->com[SIDE] = -1;
  an->terrain_index = -1;
  an->location_index = -1;
  an->parent = NULL;
  an->children = NULL;
  an->sibling = NULL;
  an->next = NULL;
  an->location_next = NULL;
  an->cost_from_start = -1;
  an->one_step_cost = -1;
  an->terrain_cost = -1;
  an->cost_to_go = -1;
  an->priority = -1;

  return an;
}

/*******************************************************************/

void free_astar_node( ASTAR *aa, ASTAR_NODE *an )
{
  if ( an == NULL )
    return;

  if ( an->parent != NULL || an->children != NULL || an->sibling != NULL
       || an->next != NULL || an->location_next != NULL )
    {
      fprintf( stderr, "Can't free ASTAR_NODE that is in use.\n" );
      exit( -1 );
    }

  if ( debug_free )
    printf( "fan: %d\n", an->id );

  an->depth = 0;

  an->next = aa->free_list;
  aa->free_list = an;

  if ( debug_free )
    printf( "fan 2: %d\n", an->id );
}

/*******************************************************************/
// traversing children, and sibling links should do it.

void free_all_astar_nodes( ASTAR *aa, ASTAR_NODE *an )
{
  ASTAR_NODE *an_next;
  
  if ( an == NULL )
    return;

  if ( debug_free )
    printf( "faan: %d\n", an->id );

  for ( ; ; )
    {
      if ( an == NULL )
	return;

      an_next = an->next;

      an->parent = NULL;
      an->sibling = NULL;
      an->children = NULL;
      an->next = NULL;
      an->location_next = NULL;

      if ( debug_free )
	printf( "faan 2: %d\n", an->id );

      free_astar_node( aa, an );

      if ( debug_free )
	printf( "faan 3\n" );

      an = an_next;
    }

  if ( debug_free )
    printf( "faan 4\n" );
}

/*******************************************************************/

void free_astar( ASTAR *aa )
{
  if ( aa == NULL )
    {
      fprintf( stderr, "NULL aa in free_astar()\n" );
      exit( -1 );
    }

  aa->active = 0;

  if ( debug_free )
    printf( "free_astar 100\n" );

  free_all_astar_nodes( aa, aa->priority_q );

  if ( debug_free )
    printf( "free_astar 110\n" );

  free_all_astar_nodes( aa, aa->done_list );

  if ( debug_free )
    printf( "free_astar 120\n" );

  free_all_astar_nodes( aa, aa->at_goal );

  if ( debug_free )
    printf( "free_astar 130\n" );

  aa->priority_q = NULL;
  aa->done_list = NULL;
  aa->at_goal = NULL;
  aa->root_node = NULL;

  if ( debug_free )
    printf( "free_astar 200\n" );
}

/*******************************************************************/

float heuristic( ASTAR *aa, ASTAR_NODE *an )
{

  if ( an->state[XX] < aa->tt->min[XX] )
    return BIG_VALUE;
  if ( an->state[YY] < aa->tt->min[YY] )
    return BIG_VALUE;
  if ( an->state[XX] > aa->tt->max[XX] )
    return BIG_VALUE;
  if ( an->state[YY] > aa->tt->max[YY] )
    return BIG_VALUE;

  if ( an->com[XX] < aa->tt->min[XX] )
    return BIG_VALUE;
  if ( an->com[YY] < aa->tt->min[YY] )
    return BIG_VALUE;
  if ( an->com[XX] > aa->tt->max[XX] )
    return BIG_VALUE;
  if ( an->com[YY] > aa->tt->max[YY] )
    return BIG_VALUE;

  // use simple distance to goal as heuristic function
  return xya_distance( aa, an->com, aa->tt->goal );
}

/*******************************************************************/
float astar_step_cost( ASTAR *aa, ASTAR_NODE *an_child, ASTAR_NODE *an )
{
  float n_s_w, d_s_l, offset, A, B, C, D;
  float p_s_o[2], c_s_o[2], f_s_o[2], p_s[2], c_s[2], f_s[2];
  float c_a, f_a; // current orientation and future orientation
  float x, y, l, s_w, ori_shift, p_s_l;
  float E_m, E_s_a, E_s, E_r;


  n_s_w = HIP_WIDTH;
  d_s_l = STEP_LENGTH;
  offset = 0.3;
  A = (1+offset)/(4*d_s_l*d_s_l*d_s_l);
  B = 3*A*d_s_l*d_s_l*d_s_l*d_s_l;
  C = 0.1;
  D = n_s_w;

  if (an == NULL)
  {
      fprintf( stderr, "Parent cannot be NULL \n" );
      exit( -1 );	
  }
  else
  {
	  c_s_o[XX] = an->state[XX];
	  c_s_o[YY] = an->state[YY];
	  if (an->parent == NULL)
	  {
		p_s_o[XX] = an->state[XX];
		p_s_o[YY] = an->state[YY];
	  }
	  else
	  {
		  p_s_o[XX] = an->parent->state[XX];
		  p_s_o[YY] = an->parent->state[YY];
	  }
  }

  f_s_o[XX] = an_child->state[XX];
  f_s_o[YY] = an_child->state[YY];

  c_a = an->state[ANGLE];
  f_a = an_child->state[ANGLE];

  p_s[XX] = p_s_o[XX]*cosf(c_a)-p_s_o[YY]*sinf(c_a);
  p_s[YY] = p_s_o[XX]*sinf(c_a)+p_s_o[YY]*cosf(c_a);
  c_s[XX] = c_s_o[XX]*cosf(c_a)-c_s_o[YY]*sinf(c_a);
  c_s[YY] = c_s_o[XX]*sinf(c_a)+c_s_o[YY]*cosf(c_a);
  f_s[XX] = f_s_o[XX]*cosf(c_a)-f_s_o[YY]*sinf(c_a);
  f_s[YY] = f_s_o[XX]*sinf(c_a)+f_s_o[YY]*cosf(c_a);

  ori_shift = f_a - c_a;
  x = f_s[XX] - p_s[XX];
  y = f_s[YY] - p_s[YY];
  p_s_l = sqrtf((c_s[YY] - p_s[YY])*(c_s[YY] - p_s[YY]) + (c_s[XX] - p_s[XX])*(c_s[XX] - p_s[XX]));

  if (p_s_l<0.1)
    l = sqrtf(x*x+y*y);
  else 
    l = sqrtf(x*x+y*y)/2*sqrtf(d_s_l/p_s_l);

  if (an->state[SIDE]==LEFT)
	s_w = (f_s[XX] - c_s[XX]);
  else
	s_w = -(f_s[XX] - c_s[XX]);

  E_m = 0.5*(A*l*l*l+B/l-offset);

  E_s_a = 0;
  if (s_w<0)
    E_s_a = powf((n_s_w-s_w)/n_s_w,0.2)*D*(n_s_w-s_w);
  else if (s_w<n_s_w)
    E_s_a = D*(n_s_w-s_w);
  
  E_s = 8*(C*s_w*s_w + E_s_a);
  E_r = 0.2*ori_shift*ori_shift/l;

  return ((E_m + E_s + E_r)/4);
	
}


/*******************************************************************/
float astar_terrain_cost( ASTAR *aa, ASTAR_NODE *an )
{
  int ix, iy, index;
  float px_c, py_c, px, py, rough, sum, max, s, c;
  float value[15];

  if ( an->state[XX] < aa->tt->min[XX] )
    return BIG_COST;
  if ( an->state[YY] < aa->tt->min[YY] )
    return BIG_COST;
  if ( an->state[XX] > aa->tt->max[XX] )
    return BIG_COST;
  if ( an->state[YY] > aa->tt->max[YY] )
    return BIG_COST;

  sum = 0;

	s = sinf( an->state[ANGLE] );
	c = cosf( an->state[ANGLE] );
	//s = 0;
	//c = 1;

  for(ix=0;ix<3;ix++)
  {
	  for(iy=0;iy<5;iy++)
	  {
		  px_c = 0.05*(ix-1);
		  py_c = 0.05*(iy-2);
		  px = px_c*c-py_c*s + an->state[XX];
		  py = px_c*s+py_c*c + an->state[YY];

		  if ( px < aa->tt->min[XX] )
			return BIG_COST;
		  if ( py < aa->tt->min[YY] )
			return BIG_COST;
		  if ( px > aa->tt->max[XX] )
			return BIG_COST;
		  if ( py > aa->tt->max[YY] )
			return BIG_COST;
		  terrain_indices( aa->tt, px, py, NULL, NULL, &index); 
		  value[5*ix+iy] = aa->tt->perceived_cost_map[index];
		  sum += value[5*ix+iy];
	  }
  }
  max = 0;
  rough = 0;
  for(ix=0;ix<15;ix++)
  {
	rough += fabsf(value[ix]-sum/15);
	if (fabsf(value[ix]-sum/15)>max)
		max = fabsf(value[ix]-sum/15);
  }
  rough = rough/15;

  if (max>0.1)
	  return BIG_COST;
  else if (rough > 0.05)
	  return BIG_COST;
  else if (sum/15>0.2)
	  return BIG_COST;
  else
	  return (float) (STEP_COST_WEIGHT*(max+2*rough+sum/15));


  //return (float) (STEP_COST_WEIGHT
		  //*(aa->tt->perceived_cost_map[ an->terrain_index ]));
}

/*******************************************************************/

ASTAR *create_astar( TERRAIN *tt )
{
  int i;
  ASTAR *aa;

  aa = (ASTAR *) malloc( sizeof( ASTAR ) );
  if ( aa == NULL )
    {
      fprintf( stderr, "Can't allocate astar.\n" );
      exit( -1 );
    }
  aa->tt = tt;

  aa->location_map =
    (ASTAR_NODE **) malloc( tt->n_cells*sizeof(ASTAR_NODE *) );
  if ( aa->location_map == NULL )
    {
      fprintf( stderr, "Can't allocate astar location map.\n" );
      exit( -1 );
    }

  aa->active = 0;
  aa->done = 0;
  aa->priority_q = NULL;
  aa->done_list = NULL;
  aa->free_list = NULL;
  aa->at_goal = NULL;
  aa->root_node = NULL;

  for ( i = 0; i < tt->n_cells; i++ )
    aa->location_map[i] = NULL;

  aa->policy = 0;
  aa->best_value = BIG_VALUE;

  aa->expand_next_node_count = 0;

  return aa;
}

/*******************************************************************/

void init_astar( ASTAR *aa )
{
  int i;
  int index;
  ASTAR_NODE *an;
  TERRAIN *tt;
  float s, c, y_offset;

  tt = aa->tt;

  // initialize data structure
  aa->done = 0;

  // aa->free_list: This is preserved across uses.
  if ( aa->priority_q != NULL || aa->done_list != NULL || aa->at_goal != NULL
       || aa->root_node != NULL )
    {
      fprintf( stderr, "Non null pointers in init_astar\n" );
      exit( -1 );
    }

  for ( i = 0; i < tt->n_cells; i++ )
    aa->location_map[i] = NULL;

  // create first node
  an = create_astar_node( aa );

  an->depth = 0;

  an->state[XX] = tt->current[XX];
  an->state[YY] = tt->current[YY];
  an->state[ANGLE] = tt->current[ANGLE];
  an->state[SIDE] = tt->current[SIDE];

  if ( an->state[SIDE] < 0.1 )
    {
      y_offset = -HIP_WIDTH;
    }
  else
    {
      y_offset = +HIP_WIDTH;
    }

  an->com[ANGLE] = an->state[ANGLE];
  s = sinf( an->state[ANGLE] );
  c = cosf( an->state[ANGLE] );
  an->com[XX] = an->state[XX] + s*y_offset/2;
  an->com[YY] = an->state[YY] - c*y_offset/2;

  terrain_indices( aa->tt, an->state[XX], an->state[YY], NULL, NULL, &index );
  if ( index < 0 || index >= tt->n_cells )
    {
      fprintf( stderr, "Current out of bounds %d %d\n", index, tt->n_cells );
      exit( -1 );
    }
  // currently all of these are the same, but eventually they may be different
  an->terrain_index = index;
  an->location_index = index;
  
  an->cost_from_start = 0.0;
  an->one_step_cost = 0.0;
  an->terrain_cost = tt->perceived_cost_map[ an->terrain_index ];
  an->cost_to_go = heuristic( aa, an );
  an->priority = an->cost_from_start + INFLATION*an->cost_to_go;

  aa->root_node = an;

  an->location_next = aa->location_map[ an->location_index ]; // should be NULL
  aa->location_map[ an->location_index ] = an;

  // enqueue root node
  aa->priority_q = an;

  aa->policy = 0;
  aa->best_value = BIG_VALUE;

  aa->expand_next_node_count = 0;

  aa->active = 1;
}

/*******************************************************************/

ASTAR_NODE *handle_duplicates( ASTAR *aa, ASTAR_NODE *an )
{
  ASTAR_NODE *an2, *an3;
  ASTAR_NODE *last_sibling = NULL;
  float d;
  int delete_an = 0;

  if ( an->children != NULL )
    {
      fprintf( stderr, "an_child should be childless at this point.\n" );
      exit( -1 );
    }

  for ( an2 = aa->location_map[ an->location_index ]; an2 != NULL;
	an2 = an2->location_next )
    {
      // printf( "hd: %d %d\n", an->id, an2->id );
      d = state_distance( aa, an->state, an2->state );
      if ( d < DUPLICATE_DISTANCE_THRESHOLD )
	{
	  if ( an->cost_from_start < an2->cost_from_start )
	    { // should replace parent pointer in an2's children with an1
	      // so they pick the better route.
	      last_sibling = NULL;
	      for ( an3 = an2->children; an3 != NULL; an3 = an3->sibling )
		{
		  an3->parent = an;
		  last_sibling = an3;
		}
	      if ( last_sibling != NULL )
		{
		  if ( last_sibling->sibling != NULL )
		    {
		      fprintf( stderr,
			       "last_sibling->sibling should be NULL at this point.\n" );
		      exit( -1 );
		    }
		  // just to be consistent, fix children pointers
		  last_sibling->sibling = an->children;
		  an->children = an2->children;
		  an2->children = NULL;
		}
	      else
		{
		  if ( an2->children != NULL )
		    {
		      fprintf( stderr,
			       "an2->children should be NULL at this point.\n" );
		      exit( -1 );
		    }
		}

	      // SHOULD UPDATE an->children costs and priorities
	      // ideally should reorder priority q, but too expensive

	      /*
	      if ( an2->cost_from_start - an->cost_from_start > 0.1 
		   && DO_PRINTF )
		printf(
		       "%g %g XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n",
		       an->cost_from_start, an2->cost_from_start );
	      */
	      delete_an = -1;
	    }
	  else if ( an->children != NULL )
	    {
	      // so this node was better than someone else in the queue.
	      // but is worse than this one.
	      // Do nothing
	      if ( delete_an > -1 )
		{
		  fprintf( stderr, "Assumption violoation 18238\n" );
		  exit( -1 );
		}
	    }
	  else
	    {
	      if ( delete_an == 0 )
		delete_an = 1;
	    }
	}
    }

  if ( delete_an == 1 )
    {
      if ( an->parent != NULL || an->children != NULL
	   || an->sibling != NULL || an->next != NULL
	   || an->location_next != NULL )
	{
	  fprintf( stderr, "ASTAR_NODE in use 1: %x, %x, %x, %x %x\n",
		   (UINT) (an->parent), (UINT) (an->children),
		   (UINT) (an->sibling), (UINT) (an->next),
		   (UINT) (an->location_next) );
	  exit( -1 );
	}
      free_astar_node( aa, an );
      return NULL;
    }

  an->location_next = aa->location_map[ an->location_index ];
  aa->location_map[ an->location_index ] = an;

  return an;
}

/*******************************************************************/

void straight_to_goal( ASTAR *aa, ASTAR_NODE *an, ASTAR_NODE *an_child, float y_offset, float width )
{
  int i, j, k, n;
  int index;
  float s, c, d;
  float foot_x, foot_y;
  float delta_x, delta_y;
  float target_x, target_y;
  float xx, yy; // spine of search
  float x, y; // actual search
  int found_it = 0;
  float best_x, best_y, best_value;
  float value;
  float actual_step_length = STEP_LENGTH;

  best_value = BIG_COST;

  // figure out direction to goal
  // and try to go in that direction.
  // if too much, limit turn?

  c = aa->tt->goal[XX] - an->com[XX];
  s = aa->tt->goal[YY] - an->com[YY];

  d = sqrtf( c*c + s*s );

  if ( d < DISTANCE/2 )
    { // Too close, don't do anything
      // hack to kill child
      an_child->state[XX] = aa->tt->min[XX] - 1;
      return;
    }

  an_child->com[ANGLE] = an_child->state[ANGLE] = atan2f( s, c );

  s = s/d;
  c = c/d;

  // rotate about COM
  foot_x = an->com[XX] - s*y_offset/2;
  foot_y = an->com[YY] + c*y_offset/2;

  delta_x = c*DISTANCE;
  delta_y = s*DISTANCE;

  if ( d < actual_step_length )
    actual_step_length = d;

  xx = target_x = foot_x + c*actual_step_length;
  yy = target_y = foot_y + s*actual_step_length;

  n = (int) (actual_step_length/DISTANCE);

  for ( i = 0; i < n; i++ )
    {
      for ( j = 0; j < n*width; j++ )
	{
	  for ( k = -1; k <= 1; k += 2 )
	    {
	      // skip duplicate case
	      if ( j == 0 && k == 1 )
		break;

	      // search perpendicular to main search direction
	      x = xx + k*j*delta_y;
	      y = yy - k*j*delta_x;

	      if ( x < aa->tt->min[XX]
		   || x > aa->tt->max[XX]
		   || y < aa->tt->min[YY]
		   || y > aa->tt->max[YY] )
		continue;

	      terrain_indices( aa->tt, x, y, NULL, NULL, &index );
	      value = aa->tt->perceived_cost_map[ index ];
	      /*
		printf( "sa %d %d %d %d: %g %g %d %g\n",
		i, j, k, n, x, y, index,
		aa->tt->perceived_cost_map[ index ] );
	      */
	      if ( value < best_value )
		{
		  best_value = value;
		  best_x = x;
		  best_y = y;
		}
	      if ( value < 0.1 )
		{
		  found_it = 1;
		  break;
		}
	    }
	  if ( found_it )
	    break;
	}
      if ( found_it )
	break;

      xx -= delta_x;
      yy -= delta_y;
    }

  if ( best_value < BIG_COST )
    {
      an_child->state[XX] = best_x;
      an_child->state[YY] = best_y;
    }
  else
    {
      an_child->state[XX] = target_x;
      an_child->state[YY] = target_y;
    }

  an_child->com[XX] = an_child->state[XX] + s*y_offset/2;
  an_child->com[YY] = an_child->state[YY] - c*y_offset/2;

  /*
    printf( "sa: %g %g %g %g %g; %g %g; %g %g %g %g %g\n",
    an->state[XX], an->state[YY], an_child->state[ANGLE], s, c, foot_x, foot_y,
    target_x, target_y,
    an_child->state[XX], an_child->state[YY],
    aa->tt->perceived_cost_map[ index ] );

    displaya();
    glutSetWindow( wa );
    glutPostRedisplay();
    printf( "press\n" );
    getchar();
  */
}

/*******************************************************************/

void turn( ASTAR *aa, ASTAR_NODE *an, ASTAR_NODE *an_child, float y_offset,
	   float angle, float width )
{
  int i, j, k, n;
  int index;
  float s, c;
  float foot_x, foot_y;
  float delta_x, delta_y;
  float target_x, target_y;
  float xx, yy; // spine of search
  float x, y; // actual search
  int found_it = 0;
  float best_x, best_y, best_value;
  float value;
  float abs_angle;

  best_value = BIG_COST;
  abs_angle = an->state[ANGLE] + angle;
  if (abs_angle>2*M_PI)
	abs_angle = abs_angle - 2*M_PI;
  else if (abs_angle<0)
	abs_angle = abs_angle + 2*M_PI;

  an_child->com[ANGLE] = an_child->state[ANGLE] = abs_angle;
  s = sinf( an_child->state[ANGLE] );
  c = cosf( an_child->state[ANGLE] );

  // rotate about COM
  foot_x = an->com[XX] - s*y_offset/2;
  foot_y = an->com[YY] + c*y_offset/2;

  delta_x = c*DISTANCE;
  delta_y = s*DISTANCE;

  xx = target_x = foot_x + c*STEP_LENGTH;
  yy = target_y = foot_y + s*STEP_LENGTH;

  n = (int) (STEP_LENGTH/DISTANCE);

  for ( i = 0; i < n; i++ )
    {
      for ( j = 0; j < n*width; j++ )
	{
	  for ( k = -1; k <= 1; k += 2 )
	    {
	      // skip duplicate case
	      if ( j == 0 && k == 1 )
		break;

	      // search perpendicular to main search direction
	      x = xx + k*j*delta_y;
	      y = yy - k*j*delta_x;

	      if ( x < aa->tt->min[XX]
		   || x > aa->tt->max[XX]
		   || y < aa->tt->min[YY]
		   || y > aa->tt->max[YY] )
		continue;

	      terrain_indices( aa->tt, x, y, NULL, NULL, &index );
	      value = aa->tt->perceived_cost_map[ index ];
	      /*
		printf( "turn %g %d %d %d %d: %g %g %d %g\n",
		angle, i, j, k, n, x, y, index, value );
	      */
	      if ( value < best_value )
		{
		  best_value = value;
		  best_x = x;
		  best_y = y;
		}
	      if ( value < 0.1 )
		{
		  found_it = 1;
		  break;
		}
	    }
	  if ( found_it )
	    break;
	}
      if ( found_it )
	break;

      xx -= delta_x;
      yy -= delta_y;
    }

  if ( best_value < BIG_COST )
    {
      an_child->state[XX] = best_x;
      an_child->state[YY] = best_y;
    }
  else
    {
      an_child->state[XX] = target_x;
      an_child->state[YY] = target_y;
    }

  an_child->com[XX] = an_child->state[XX] + s*y_offset/2;
  an_child->com[YY] = an_child->state[YY] - c*y_offset/2;

  /*
    printf( "turn: %g %g %g %g %g %g; %g %g; %g %g %g %g %g\n", angle,
    an->state[XX], an->state[YY], an_child->state[ANGLE], s, c,
    foot_x, foot_y, target_x, target_y,
    an_child->state[XX], an_child->state[YY],
    aa->tt->perceived_cost_map[ index ] );

    displaya();
    glutSetWindow( wa );
    glutPostRedisplay();
    printf( "press\n" );
    getchar();
  */
}

/*******************************************************************/
/*******************************************************************/

ASTAR_NODE *generate_1_child( ASTAR *aa, ASTAR_NODE *an, int *i_child )
{
  ASTAR_NODE *an_child;
  float y_offset;

  // stop after 1
  if ( *i_child > 0 || *i_child < 0 )
    {
      *i_child = -1;
      return NULL;
    }

  an_child = create_astar_node( aa );

  an_child->depth = an->depth + 1;
  if ( an->state[SIDE] < 0.1 )
    {
      an_child->state[SIDE] = RIGHT;
      y_offset = HIP_WIDTH;
    }
  else
    {
      an_child->state[SIDE] = LEFT;
      y_offset = -HIP_WIDTH;
    }

  switch ( *i_child )
    {
    case 0:
      straight_to_goal( aa, an, an_child, y_offset, 1.0f );
      break;
    default:
      fprintf( stderr, "unknown n_child: %d\n", *i_child );
      exit( -1 );
    }

  (*i_child)++;

  return an_child;
}

/*******************************************************************/

ASTAR_NODE *generate_4_children( ASTAR *aa, ASTAR_NODE *an, int *i_child )
{
  ASTAR_NODE *an_child;
  float y_offset;

  // stop after 3
  if ( *i_child > 3 || *i_child < 0 )
    {
      *i_child = -1;
      return NULL;
    }

  an_child = create_astar_node( aa );

  an_child->depth = an->depth + 1;
  if ( an->state[SIDE] < 0.1 )
    {
      an_child->state[SIDE] = RIGHT;
      y_offset = HIP_WIDTH;
    }
  else
    {
      an_child->state[SIDE] = LEFT;
      y_offset = -HIP_WIDTH;
    }

  switch ( *i_child )
    {
    case 0:
      straight_to_goal( aa, an, an_child, y_offset, 0.5f );
      break;
    case 1:
      // turn right roughly 60 deg
      turn( aa, an, an_child, y_offset, -1.0f, 0.5f );
      break;
    case 2:
      // go straight
      turn( aa, an, an_child, y_offset, 0.0f, 0.5f );
      break;
    case 3:
      // turn left roughly 60 deg
      turn( aa, an, an_child, y_offset, 1.0f, 0.5f );
      break;
    default:
      fprintf( stderr, "unknown n_child: %d\n", *i_child );
      exit( -1 );
    }

  (*i_child)++;

  return an_child;
}

/*******************************************************************/

ASTAR_NODE *generate_8_children( ASTAR *aa, ASTAR_NODE *an, int *i_child )
{
  ASTAR_NODE *an_child;
  float y_offset;

  // stop after 15
  if ( *i_child > 7 || *i_child < 0 )
    {
      *i_child = -1;
      return NULL;
    }

  an_child = create_astar_node( aa );
  an_child->depth = an->depth + 1;
  if ( an->state[SIDE] < 0.1 )
    {
      an_child->state[SIDE] = RIGHT;
      y_offset = HIP_WIDTH;
    }
  else
    {
      an_child->state[SIDE] = LEFT;
      y_offset = -HIP_WIDTH;
    }

  switch ( *i_child )
    {
    case 0:
      straight_to_goal( aa, an, an_child, y_offset, 0.1f );
      break;
    case 1:
      turn( aa, an, an_child, y_offset, 0.0f,  0.1f ); // go straight
      break;
    case 2:
      turn( aa, an, an_child, y_offset, -0.2f,  0.1f );
      break;
    case 3:
      turn( aa, an, an_child, y_offset, 0.2f,  0.1f );
      break;
    case 4:
      turn( aa, an, an_child, y_offset, -0.5f,  0.2f );
      break;
    case 5:
      turn( aa, an, an_child, y_offset, 0.5f,  0.2f );
      break;
    case 6:
      turn( aa, an, an_child, y_offset, -1.1f,  0.4f );
      break;
    case 7:
      turn( aa, an, an_child, y_offset, 1.1f,  0.4f );
      break;
    default:
      fprintf( stderr, "unknown n_child: %d\n", *i_child );
      exit( -1 );
    }

  (*i_child)++;

  return an_child;
}

/*******************************************************************/

ASTAR_NODE *generate_16_children( ASTAR *aa, ASTAR_NODE *an, int *i_child )
{
  ASTAR_NODE *an_child;
  float y_offset;

  // stop after 15
  if ( *i_child > 15 || *i_child < 0 )
    {
      *i_child = -1;
      return NULL;
    }

  an_child = create_astar_node( aa );
  an_child->depth = an->depth + 1;
  if ( an->state[SIDE] < 0.1 )
    {
      an_child->state[SIDE] = RIGHT;
      y_offset = HIP_WIDTH;
    }
  else
    {
      an_child->state[SIDE] = LEFT;
      y_offset = -HIP_WIDTH;
    }

  switch ( *i_child )
    {
    case 0:
      straight_to_goal( aa, an, an_child, y_offset, 0.1f );
      break;
    case 1:
      turn( aa, an, an_child, y_offset, 0.0f,  0.1f ); // go straight
      break;
    case 2:
      turn( aa, an, an_child, y_offset, -0.2f,  0.1f );
      break;
    case 3:
      turn( aa, an, an_child, y_offset, 0.2f,  0.1f );
      break;
    case 4:
      turn( aa, an, an_child, y_offset, -0.4f,  0.1f );
      break;
    case 5:
      turn( aa, an, an_child, y_offset, 0.4f,  0.1f );
      break;
    case 6:
      turn( aa, an, an_child, y_offset, -0.6f,  0.1f );
      break;
    case 7:
      turn( aa, an, an_child, y_offset, 0.6f,  0.1f );
      break;
    case 8:
      turn( aa, an, an_child, y_offset, -0.8f,  0.1f );
      break;
    case 9:
      turn( aa, an, an_child, y_offset, 0.8f,  0.1f );
      break;
    case 10:
      turn( aa, an, an_child, y_offset, -1.0f,  0.1f );
      break;
    case 11:
      turn( aa, an, an_child, y_offset, 1.0f,  0.1f );
      break;
    case 12:
      turn( aa, an, an_child, y_offset, -1.2f,  0.1f );
      break;
    case 13:
      turn( aa, an, an_child, y_offset, 1.2f,  0.1f );
      break;
    case 14:
      turn( aa, an, an_child, y_offset, -1.4f,  0.1f );
      break;
    case 15:
      turn( aa, an, an_child, y_offset, 1.4f,  0.1f );
      break;
    default:
      fprintf( stderr, "unknown n_child: %d\n", *i_child );
      exit( -1 );
    }

  (*i_child)++;

  return an_child;
}

/*******************************************************************/

ASTAR_NODE *generate_child( ASTAR *aa, ASTAR_NODE *an, int *i_child )
{
  int index;
  ASTAR_NODE *an_child;

  // printf( "gnn: %d %d\n", an->id, *i_child );

  // allow different search policies
  if ( aa->policy == 0 )
    {
      an_child = generate_1_child( aa, an, i_child );
    }
  else if ( aa->policy == 1 )
    {
      an_child = generate_4_children( aa, an, i_child );
    }
  else if ( aa->policy == 2 )
    {
      // allow different numbers of actions at different steps
      if ( an->depth == 0 )
	an_child = generate_8_children( aa, an, i_child );
      else 
	an_child = generate_4_children( aa, an, i_child );
    }
  else 
    {
      // allow different numbers of actions at different steps
      if ( an->depth == 0 )
	an_child = generate_16_children( aa, an, i_child );
      else if ( an->depth == 1 )
	an_child = generate_8_children( aa, an, i_child );
      else
	an_child = generate_4_children( aa, an, i_child );
    }

  if ( an_child == NULL )
    {
      // printf( "gnn: NULL child\n" );
      return NULL;
    }

  // printf( "gnn: %d %d %d\n", an->id, an_child->id, *i_child );

  // out of bounds? This is useful, but also a hack to reject failed tries
  if ( an_child->state[XX] < aa->tt->min[XX]
       || an_child->state[XX] > aa->tt->max[XX]
       || an_child->state[YY] < aa->tt->min[YY]
       || an_child->state[YY] > aa->tt->max[YY]
       || an_child->com[XX] < aa->tt->min[XX]
       || an_child->com[XX] > aa->tt->max[XX]
       || an_child->com[YY] < aa->tt->min[YY]
       || an_child->com[YY] > aa->tt->max[YY] )
    {
      if ( an_child->parent != NULL || an_child->children != NULL
	   || an_child->sibling != NULL || an_child->next != NULL
	   || an_child->location_next != NULL )
		{
		  fprintf( stderr, "ASTAR_NODE in use 2: %x, %x, %x, %x %x\n",
			   (UINT) (an->parent), (UINT) (an->children),
			   (UINT) (an->sibling), (UINT) (an->next),
			   (UINT) (an->location_next) );
		  exit( -1 );
		}

      free_astar_node( aa, an_child );
      // printf( "gnn: NULL child 2\n" );
      return NULL;
    }

  // printf( "gnn 2: %d %d %d\n", an->id, an_child->id, *i_child );

  terrain_indices( aa->tt, an_child->state[XX], an_child->state[YY],
		   NULL, NULL, &index );
  if ( index < 0 || index >= aa->tt->n_cells )
    {
      fprintf( stderr, "State out of bounds %d %d; %g %g %g\n", index,
	       aa->tt->n_cells, an_child->state[XX], an_child->state[YY],
	       an_child->state[ANGLE] );
      exit( -1 );
    }
  // currently all of these are the same, but eventually they may be different
  an_child->terrain_index = index;
  an_child->location_index = index;

  // printf( "gnn 3: %d %d %d\n", an->id, an_child->id, *i_child );

  //an_child->one_step_cost = xya_distance( aa, an->state, an_child->state );

  an_child->one_step_cost = astar_step_cost( aa, an_child, an);

  // printf( "gnn 4: %d %d %d\n", an->id, an_child->id, *i_child );

  an_child->terrain_cost = astar_terrain_cost( aa, an_child );

  // printf( "gnn 5: %d %d %d\n", an->id, an_child->id, *i_child );

  an_child->cost_from_start = an->cost_from_start
    + an_child->one_step_cost + an_child->terrain_cost;

  // printf( "gnn 6: %d %d %d\n", an->id, an_child->id, *i_child );

  an_child->cost_to_go = heuristic( aa, an_child );

  // printf( "gnn 7: %d %d %d\n", an->id, an_child->id, *i_child );

  an_child->priority
    = an_child->cost_from_start + INFLATION*an_child->cost_to_go;
  // could implement a too expensive limit (although this just goes on
  // end of prioity queue)

  // printf( "gnn: before hd: %d %d %d\n", an->id, an_child->id, *i_child );

  // check for duplicates
  an_child = handle_duplicates( aa, an_child );

  if ( an_child == NULL )
    {
      // printf( "gnn: NULL child 3\n" );
      return NULL;
    }

  // printf( "gnn: after hd: %d %d %d\n", an->id, an_child->id, *i_child );

  an_child->parent = an;
  an_child->sibling = an->children;
  an->children = an_child;

  return an_child;
}

/*******************************************************************/

void enqueue( ASTAR *aa, ASTAR_NODE *an )
{
  int j;
  ASTAR_NODE **prev;
  ASTAR_NODE *q;

  if ( an == NULL )
    return;

  if ( aa->priority_q == NULL )
    {
      /*
	fprintf( stderr, "NULL Priority Queue\n" );
	exit( -1 );
      */
      aa->priority_q = an;
      return;
    }

  prev = &(aa->priority_q);
  q = aa->priority_q; 

  for ( j = 0; ; j++ )
    {
      if ( q == NULL )
	{
	  an->next = q;
	  *prev = an;
	  return;
	}
	  
      if ( q->priority >= an->priority )
	{
	  an->next = q;
	  *prev = an;
	  return;
	}

      prev = &(q->next);
      q = q->next;

      if ( j > 1000000 )
	{
	  fprintf( stderr, "enqueue: loop broken.\n" );
	  exit( -1 );
	}
    }
}

/*******************************************************************/

void printf_priority_q( ASTAR *aa )
{
  ASTAR_NODE *an;
  int count = 0;

  if ( !DO_PRINTF )
    return;

  an = aa->priority_q;
  // printf( "priority_queue: " );
  for ( ; ; )
    {
      if ( an == NULL )
	break;
      // printf( "%d, ", an->id );
      count++;
      an = an->next;
    }
  // printf( "\n" );
  printf( "Priority queue: %d nodes.\n", count );
}

/*******************************************************************/

void printf_done_list( ASTAR *aa )
{
  ASTAR_NODE *an;
  int count = 0;

  if ( !DO_PRINTF )
    return;

  an = aa->done_list;
  // printf( "done_list: " );
  for ( ; ; )
    {
      if ( an == NULL )
	break;
      // printf( "%d, ", an->id );
      count++;
      an = an->next;
    }
  // printf( "\n" );
  printf( "Done list: %d nodes.\n", count );
}

/*******************************************************************/

void printf_at_goal( ASTAR *aa )
{
  ASTAR_NODE *an;
  int count = 0;

  if ( !DO_PRINTF )
    return;

  an = aa->at_goal;
  // printf( "at_goal_list: " );
  for ( ; ; )
    {
      if ( an == NULL )
	break;
      // printf( "%d, ", an->id );
      count++;
      an = an->next;
    }
  // printf( "\n" );
  printf( "At_goal list: %d nodes.\n", count );
}

/*******************************************************************/

void printf_free_list( ASTAR *aa )
{
  ASTAR_NODE *an;
  int count = 0;

  if ( !DO_PRINTF )
    return;

  an = aa->free_list;
  // printf( "free_list: " );
  for ( ; ; )
    {
      if ( an == NULL )
	break;
      // printf( "%d, ", an->id );
      count++;
      an = an->next;
    }
  // printf( "\n" );
  printf( "Free list: %d nodes. %d ids\n", count, astar_node_count );
}

/*******************************************************************/

void printf_path( ASTAR *aa )
{
  ASTAR_NODE *an;
  int count = 0;

  if ( !DO_PRINTF )
    return;

  an = aa->at_goal;
  printf( "path:\n" );
  for ( ; ; )
    {
      if ( an == NULL )
	break;
      printf( "%d %d: %g %g %g %g; %g %g %g %g\n",
	      an->id, an->depth, an->state[XX], an->state[YY],
	      an->state[ANGLE], an->state[SIDE],
	      an->cost_from_start, an->cost_to_go, an->priority,
	      an->terrain_cost );
      count++;
      an = an->parent;
    }
  // printf( "path: %d nodes.\n", count );
}

/*******************************************************************/

void printf_final_path( PATH_NODE *pn )
{
  int count = 0;
  PATH_NODE *pn_original;

  pn_original = pn;

  if ( DO_PRINTF )
    printf( "final path from goal:\n" );
  for ( ; ; )
    {
      if ( pn == NULL )
	break;
      if ( DO_PRINTF )
	printf( "%3d: %7.3f %7.3f %7.3f %2.0f; %8.3f %8.3f %8.3f %8.3f %8.3f\n",
		pn->id, pn->state[XX], pn->state[YY], pn->state[ANGLE],
		pn->state[SIDE],
		pn->cost_from_start, pn->cost_to_go, pn->priority,
		pn->one_step_cost,
		// pn->cost_from_start - pn->one_step_cost - pn->terrain_cost,
		pn->terrain_cost );
      count++;
      pn = pn->previous;
    }
  printf( "Final path: cost %g; %d nodes.\n", pn_original->cost_from_start, count );
}

/*******************************************************************/

void expand_next_node( ASTAR *aa, ASTAR_NODE *an )
{
  int i;
  int j;
  ASTAR_NODE *an2;
  ASTAR_NODE *child;

  if ( an == NULL )
    return;

  if ( debug_a )
    printf( "expand_next_node: %d %g %g\n", an->id,
	    an->state[XX], an->state[YY] );

  for ( i = 0; ; )
    {
      // printf( "enn: %d %d\n", an->id, i );

      // stop making children
      if ( i < 0 )
		break;

      child = generate_child( aa, an, &i );

      // printf( "ennx: %d %d\n", an->id, i );

      if ( child == NULL )
		continue;

      if ( child->next != NULL )
		{
		fprintf( stderr, "Expected child's next to be NULL\n" );
		exit( -1 );
		}

      if ( ( aa->tt->known_terrain[ child->terrain_index ] < 0.1 )
	   // kill stuff that is in unknown territory away from goal
	   && ( child->cost_to_go > aa->root_node->cost_to_go ) )
		{
		child->next = aa->done_list;
		aa->done_list = child;
		}
      else if ( (child->cost_to_go < CLOSE_ENOUGH )
		|| ( aa->tt->known_terrain[ child->terrain_index ] < 0.1 ) )
		{
		if ( aa->at_goal == NULL )
			printf( "enn: %d %g\n", aa->expand_next_node_count,
		    child->priority );
		else if ( aa->at_goal->priority > child->priority )
			printf( "enn: %d %g %g\n", aa->expand_next_node_count,		
			aa->at_goal->priority, child->priority );

		// manage best so far
		if ( aa->best_value > child->priority )
			aa->best_value = child->priority;

		if ( aa->at_goal == NULL )
			{
			// printf( "enn: NULL at_goal\n" );
			child->next = NULL;
			aa->at_goal = child;
			}
		else if ( aa->at_goal->priority > child->priority )
			{
			/*
			printf( "enn: new guy better: %g %g\n", child->priority,
		      aa->at_goal->priority );
			*/
			child->next = aa->at_goal;
			aa->at_goal = child;
			}
		else
			{
			an2 = aa->at_goal;
			for( j = 0; ; j++ )
				{
				/*
				if ( an2->next == NULL )
				printf( "an2: %d %d\n", an2->id, j );
				else
				printf( "an2: %d %d %d\n", an2->id, an2->next->id, j );
				*/
				if ( an2->next == NULL)
					{
					// printf( "enn: last guy in queue\n" );
					an2->next = child;
					break;
					}
				if ( an2->next->priority > child->priority )
					{
					/*
					printf( "enn: new guy better: %g %g\n", child->priority,
					  an2->next->priority );
					*/
					child->next = an2->next;
					an2->next = child;
					break;
					}
				an2 = an2->next;
		  
				if ( j > 1000000 )
					{
					fprintf( stderr, "enn: loop broken.\n" );
					exit( -1 );
					}
				}
			}
		if ( QUIT_ON_FIRST_SUCCESS )
			{
			aa->done = 1;
			if ( DO_PRINTF )
				{
				/*
				printf( "Done.\n" );
				printf_priority_q( aa );
				printf_done_list( aa );
				printf_path( aa );
				*/
				}
			return;
			}
		}
      else
		{
		enqueue( aa, child );
		}
    }

  if ( debug_a )
    printf( "expand_next_node done: %d\n", an->id );

  /*
    displaya();
    glutSetWindow( wa );
    glutPostRedisplay();
    printf( "press\n" );
    getchar();
  */
}

/******* PATH ****************************************************/

PATH_NODE* create_path_node( ASTAR_NODE *an )
{
  int i;
  PATH_NODE *pn;
  static int path_node_count = 0;

  pn = (PATH_NODE *) malloc( sizeof( PATH_NODE ) );

  if ( pn == NULL )
    {
      fprintf( stderr, "Can't allocate path node.\n" );
      exit( -1 );
    }
  pn->id = path_node_count++;
  pn->an_id = an->id;
  for ( i = 0; i < N_X; i++ )
    pn->state[i] = an->state[i];
  pn->previous = NULL;
  pn->next = NULL;
  pn->cost_from_start = an->cost_from_start;
  pn->cost_to_go = an->cost_to_go;
  pn->one_step_cost = an->one_step_cost;
  pn->terrain_cost = an->terrain_cost;
  pn->priority = an->priority;

  return pn;
}

/*******************************************************************/

void add_an_to_path( ASTAR *aa, ASTAR_NODE *an )
{
  PATH_NODE *pn;

  pn = create_path_node( an );
      
  pn->previous = p1;
  if ( p1 != NULL )
    {
      p1->next = pn;
      pn->cost_from_start += p1->cost_from_start;
    }
  p1 = pn;
  

  if ( DO_PRINTF )
    printf( "path: %g %g %g %g; %g %g %g %g %g %g\n",
	    p1->state[XX], p1->state[YY], p1->state[ANGLE], p1->state[SIDE],
	    p1->cost_from_start, p1->cost_to_go, p1->priority,
	    p1->one_step_cost,
	    p1->cost_from_start + p1->one_step_cost + p1->terrain_cost,
	    p1->terrain_cost );

  if ( p1->cost_to_go < CLOSE_ENOUGH )
    {
      if ( DO_PRINTF )
	printf( "Goal reached.\n" );
      path_done = 1;
    }
}

/******* PERCEPTION ****************************************************/

void perceive( TERRAIN *tt )
{
  int ixc, iyc;
  int ix, iy;
  float value;

  terrain_indices( tt, tt->current[XX], tt->current[YY], &ixc, &iyc, NULL );

  for ( ix = ixc - tt->perception_radius;
	ix - ixc <= tt->perception_radius;
	ix++ )
    {
      for ( iy = iyc - tt->perception_radius;
	    iy - iyc < tt->perception_radius;
	    iy++ )
	{
	  if ( ((ix-ixc)*(ix-ixc) + (iy-iyc)*(iy-iyc)) <
	       (tt->perception_radius)*(tt->perception_radius) )
	    {
	      value = get_true_cost_map_pixel( tt, ix, iy );
	      set_perceived_cost_map_pixel( tt, ix, iy, value );
	    }
	}
    }
}

/******* PLANNING  ****************************************************/

int plan_count = 0;

void plan()
{
  int j;
  ASTAR_NODE *an;
  static int first_time = 1;

  printf( "plan: %d\n", ++plan_count );

  /*
  if ( plan_count > 13 )
    debug_plan = 1;
  */

  free_astar( a1 );

  // printf_free_list( a1 );

  if ( debug_plan )
    printf( "p105 %d\n", plan_count );

  perceive( t1 );

  if ( debug_plan )
    printf( "p110 %d\n", plan_count );

  init_astar( a1 );

  for( a1->expand_next_node_count = 0; a1->expand_next_node_count < 5000;
       a1->expand_next_node_count++ )
    {
      if ( a1->done ){
		printf( "End 1\n");
		break;	
	  }

      if ( a1->priority_q == NULL )
		{
		// a1->done = 1;
		printf( "NULL priority_q: %d\n", a1->expand_next_node_count );
		a1->policy++;
		if ( a1->policy >= 4 )
			{
			a1->done = 1;
			printf( "End 2\n");
			break;
			}
		else
			expand_next_node( a1, a1->root_node );
		continue;
		}

      an = a1->priority_q;
      a1->priority_q = an->next;
      an->next = a1->done_list;
      a1->done_list = an;

      if ( an->priority > a1->best_value + CUTOFF_INCREMENT )
		{
		// this node doesn't count
		a1->expand_next_node_count--;
		continue;
		}

      expand_next_node( a1, an );
    }
  printf( "%d node expansions.\n", a1->expand_next_node_count );

  if ( debug_plan )
    printf( "p120 %d\n", plan_count );


  // Okay, use best termination
  if ( a1->at_goal ) // this is good
    {
      an = a1->at_goal;
      if ( DO_PRINTF )
		printf( "****GOOD****** planning using at_goal node\n" );
    }
  else if ( a1->priority_q ) // this is less good
    {
      an = a1->priority_q;
      if ( DO_PRINTF )
		printf( "****OK******* planning using priority_q node\n" );
    }
  else
    {
      an = a1->done_list; // this is really bad
      if ( DO_PRINTF )
		printf( "****BAD******* planning using done_list node\n" );
    }

  if ( an == NULL )
    {
      fprintf( stderr, "Null search.\n" );
      exit( -1 );
    }

  if ( debug_plan )
    printf( "p130 %d\n", plan_count );

  // Find starting step
  for ( j = 0; ; j++ )
    {
      if ( an->parent == NULL ) // this is really really bad, 1 node only
		{
		  path_done = 1;
		  if ( DO_PRINTF )
			printf( "Unable to make a plan that takes 1 step.\n" );
		  return; // we give up
		}
      if ( an->parent->parent == NULL )
		break;
	  add_an_to_path( a1, an );
      an = an->parent;
      if ( j > 1000000 )
		{
		fprintf( stderr, "plan: loop broken.\n" );
		exit( -1 );
		}
    }

  add_an_to_path( a1, an );

  if ( first_time )
    {
      if ( an->parent == NULL )
		{
		fprintf( stderr, "Null parent???\n" );
		exit( -1 );
		}
      add_an_to_path( a1, an->parent );
    }
  first_time = 0;

  if ( debug_plan )
    printf( "p200 %d\n", plan_count );
}

/******* IDLE ****************************************************/
// This subroutine actually does all the work

void my_idlea( void )
{
  int i;
  static int first_time = 1;
  static int printed_path = 0;

  if ( !path_done )
    plan();

  if ( !path_done  )
    {
      /*
      if ( DO_PRINTF )
	{
	  printf(
	 "Press return to continue. (click on this window if no response)\n" );
	  getchar();
	}
      */

		// Move to next state
		for ( i = 0; i < N_X; i++ )
			t1->current[i] = p1->state[i];
	}
  else if ( !printed_path )
    {
      printf_final_path( p1 );
      printed_path = 1;
    }

  first_time = 0;
}

void generate_true_cost_map_old( TERRAIN *tt, unsigned int seed )
{
  int i, j;
  // int ir;
  int ir2;
  int ix, ixc, ixl;
  int iy, iyc, iyl;
  int ixc2, iyc2;
  int kx, ky;
  float value[TERRAIN_N_X][TERRAIN_N_Y];
  float filter_coeff = 0.6f;

  tt->min[XX] = 0.0;
  tt->min[YY] = 0.0;
  tt->max[XX] = 25.0;
  tt->max[YY] = 25.0;

  tt->inc = (tt->max[XX] - tt->min[XX])/TERRAIN_N_X;
  if ( fabsf( tt->inc - (tt->max[YY] - tt->min[YY])/TERRAIN_N_Y ) > 1e-3 )
    {
      // Need to implement more sophisticated tt->inc 
      fprintf( stderr, "X and Y distance/pixel not equal: %g %g\n",
	       tt->inc, (tt->max[YY] - tt->min[YY])/TERRAIN_N_Y );
      exit( -1 );
    }

  // See how far away perception horizon is.
  terrain_indices( tt, 0.0f, 0.0f, &ixc, &iyc, NULL );
  terrain_indices( tt, 6.0f, 6.0f, &ixc2, &iyc2, NULL );
  tt->perception_radius = ixc2 - ixc;

  // current and goal states
  tt->current[XX] = 1;
  tt->current[YY] = 1;
  tt->current[ANGLE] = (float) (M_PI/4);
  tt->current[SIDE] = LEFT;

  tt->goal[XX] = 24;
  tt->goal[YY] = 24;
  tt->goal[ANGLE] = tt->current[ANGLE];
  // should ignore this in calculating distance to goal
  tt->current[SIDE] = LEFT; 

  // clear cost map
  for ( i = 0; i < tt->resolution[XX]; i++ )
    for ( j = 0; j < tt->resolution[YY]; j++ )
      set_true_cost_map_pixel( tt, i, j, 0.0 );
    

  // change seed to get new terrain
  srand( seed ); 

  
  // random rectangles: this is in pixel units
  for ( i = 0; i < 100; i++ )
  {
  ixc = rand() % tt->resolution[XX];
  iyc = rand() % tt->resolution[YY];
  ixl = rand() % 50; // 25 is good // 30 requires errors
  iyl = rand() % 50;
  for ( ix = ixc;  ix <= ixc + ixl; ix++ )
  for ( iy = iyc; iy <= iyc + iyl; iy++ )
	set_true_cost_map_pixel( tt, ix, iy, 1.0 );
  }

  // random circles: this is in pixel units
  for ( i = 0; i < 50; i++ )
    {
      ixc = rand() % tt->resolution[XX];
      iyc = rand() % tt->resolution[YY];
      ir2 = rand() % 900;
      ixl = iyl = (int) sqrtf( (float) ir2 );
      for ( ix = ixc - ixl;  ix <= ixc + ixl; ix++ )
	for ( iy = iyc - iyl; iy <= iyc + iyl; iy++ )
	  {
	    if ( (ix - ixc)*(ix - ixc) + (iy - iyc)*(iy - iyc) <= ixl*ixl )
	      set_true_cost_map_pixel( tt, ix, iy, 1.0 );
	  }
    }

  // random bars: this is in pixel units
  for ( i = 0; i < 100; i++ )
  {
  ixc = rand() % tt->resolution[XX];
  iyc = rand() % tt->resolution[YY];
  ixl = rand() % 50; 
  iyl = 2;
  for ( ix = ixc;  ix <= ixc + ixl; ix++ )
  for ( iy = iyc; iy <= iyc + iyl; iy++ )
	set_true_cost_map_pixel( tt, ix, iy, 1.0 );
  }

  // free goal
  terrain_indices( tt, tt->goal[XX], tt->goal[YY], &ixc, &iyc, NULL );
  ixl = iyl = 10;
  for ( ix = ixc - ixl;  ix <= ixc + ixl; ix++ )
    for ( iy = iyc - iyl; iy <= iyc + iyl; iy++ )
      {
	if ( (ix - ixc)*(ix - ixc) + (iy - iyc)*(iy - iyc) <= ixl*ixl )
	  set_true_cost_map_pixel( tt, ix, iy, 0.0 );
      }

  // free start
  terrain_indices( tt, tt->current[XX], tt->current[YY], &ixc, &iyc, NULL );
  ixl = iyl = 10;
  for ( ix = ixc - ixl;  ix <= ixc + ixl; ix++ )
    for ( iy = iyc - iyl; iy <= iyc + iyl; iy++ )
      {
	if ( (ix - ixc)*(ix - ixc) + (iy - iyc)*(iy - iyc) <= ixl*ixl )
	  set_true_cost_map_pixel( tt, ix, iy, 0.0 );
      }
   	
  // BLUR COST MAP
  // blur each column
  for ( ix = 0; ix < tt->resolution[XX]; ix++ )
    { 
      for ( iy = 0; iy < tt->resolution[YY]; iy++ )
		{
			value[ix][iy] = 0;
            for (ky = -2; ky <= 2; ky++)
                for (kx = -2; kx <= 2; kx++)
                    value[ix][iy] += get_true_cost_map_pixel(tt,ix + kx, iy + ky);
			value[ix][iy] = value[ix][iy]/25;
			
		}
    }
  for ( ix = 0; ix < tt->resolution[XX]; ix++ )
	  for ( iy = 0; iy < tt->resolution[YY]; iy++ )
		set_true_cost_map_pixel( tt, ix, iy, value[ix][iy]);
	
}
