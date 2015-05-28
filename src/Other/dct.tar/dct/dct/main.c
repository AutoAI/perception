/*   Author: Geoff Leach, Department of Computer Science, RMIT.
 *   email: gl@cs.rmit.edu.au
 *
 *   Date: 6/10/93
 *
 *   Version 1.0
 *   
 *   Copyright (c) RMIT 1993. All rights reserved.
 *
 *   License to copy and use this software purposes is granted provided 
 *   that appropriate credit is given to both RMIT and the author.
 *
 *   License is also granted to make and use derivative works provided
 *   that appropriate credit is given to both RMIT and the author.
 *
 *   RMIT makes no representations concerning either the merchantability 
 *   of this software or the suitability of this software for any particular 
 *   purpose.  It is provided "as is" without express or implied warranty 
 *   of any kind.
 *
 *   These notices must be retained in any copies of any part of this software.
 */

#include  "defs.h"
#include  "decl.h"
#include  "extern.h"
#include  "edge.h"
#include  <stdio.h>
#include  <stdlib.h>

int main(int argc, char *argv[])
{
  cardinal n;
  edge *l_cw, *r_ccw;
  index i;
  point **p_sorted, **p_temp;

  if (scanf("%d", &n) != 1)
    panic("Problem reading number of points on first line\n");

  if (n <= 0)
    panic("Number of points has to be greater than 0\n");

  alloc_memory(n);

  read_points(n);

  /* Initialise entry edge pointers. */
  for (i = 0; i < n; i++)
    p_array[i].entry_pt = NULL;

  /* Sort. */
  p_sorted = (point **)malloc((unsigned)n*sizeof(point *));
  if (p_sorted == NULL)
    panic("triangulate: not enough memory\n");
  p_temp = (point **)malloc((unsigned)n*sizeof(point *));
  if (p_temp == NULL)
    panic("triangulate: not enough memory\n");
  for (i = 0; i < n; i++)
    p_sorted[i] = p_array + i;

  merge_sort(p_sorted, p_temp, 0, n-1);
  
  free((char *)p_temp);


  /* Triangulate. */
  divide(p_sorted, 0, n-1, &l_cw, &r_ccw);

  free((char *)p_sorted);

  if (argc == 2)
    print_results(n, argv[1][1]);
  else
    print_results(n, 'e');

  free_memory();

  exit(EXIT_SUCCESS);
  return 0;	/* To keep lint quiet. */
}
