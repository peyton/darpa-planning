#include "demo.h"

/******* MAIN ****************************************************/

int main( int argc, char** argv )
{
  unsigned int my_seed = 1;

  t1 = create_terrain();

  if ( argc > 1 )
    my_seed = atoi( argv[1] );
  if ( DO_PRINTF )
    printf( "%s %d: seed: %u\n", argv[0], argc, my_seed );

  generate_true_cost_map( t1, my_seed );

  a1 = create_astar( t1 );

  init_graphics( &argc, argv, t1 );

#ifdef DO_GRAPHICS
  glutMainLoop();
#else
  for( ; ; )
    {
	#ifndef DO_GRAPHICS
		if ( path_done )
			return 0;
	#endif
    my_idlea();
    }
#endif
  return 0;
}
