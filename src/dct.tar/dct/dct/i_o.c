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

static void print_edges(cardinal n);
static void print_triangles(cardinal n);

void read_points(cardinal np)
{
  index i;

  for (i = 0; i < np; i++)
    if (scanf("%f %f", &p_array[i].x, &p_array[i].y) != 2)
	panic("Error reading points\n");
}

/*
 * Driver function.
 */
void print_results(cardinal n, char o)
{
  /* Output edges or triangles. */
  if (o == 't')
    print_triangles(n);
  else
    print_edges(n);
}

/* 
 *  Print the ring of edges about each vertex.
 */
static void print_edges(cardinal n)
{
  edge *e_start, *e;
  point *u, *v;
  index i;

  for (i = 0; i < n; i++) {
    u = &p_array[i];
    e_start = e = u->entry_pt;
    do
    {
      v = Other_point(e, u);
      if (u < v)
	if (printf("%d %d\n", u - p_array, v - p_array) == EOF)
	  panic("Error printing results\n");
      e = Next(e, u);
    } while (!Identical_refs(e, e_start));
  }
}

/* 
 *  Print the ring of triangles about each vertex.
 */
static void print_triangles(cardinal n)
{
  edge *e_start, *e, *next;
  point *u, *v, *w;
  index i;
  point *t;

  for (i = 0; i < n; i++) {
    u = &p_array[i];
    e_start = e = u->entry_pt;
    do
    {
      v = Other_point(e, u);
      if (u < v) {
	next = Next(e, u);
	w = Other_point(next, u);
	if (u < w)
	  if (Identical_refs(Next(next, w), Prev(e, v))) {  
	    /* Triangle. */
	    if (v > w) { t = v; v = w; w = t; }
	    if (printf("%d %d %d\n", u - p_array, v - p_array, w - p_array) == EOF)
	      panic("Error printing results\n");
	  }
      }

      /* Next edge around u. */
      e = Next(e, u);
    } while (!Identical_refs(e, e_start));
  }
}
