/*
    Copyright (C) 2013 Jordan Hargrave<jordan_hargrave@hotmail.com>

    osm.c - utility for creating/reading OpenStreetMap files

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#define _LARGEFILE_SOURCE
#define FILE_OFFSET_BITS  64

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/queue.h>
#include <memory.h>
#include "bmp.h"

struct node {
  uint64_t id;
  double   lat,lon;
  SLIST_ENTRY(node) entry;
};

SLIST_HEAD(,node) nlist;

struct node *findnode(uint64_t id)
{
  struct node *node;

  SLIST_FOREACH(node, &nlist, entry) {
    if (node->id == id)
      return node;
  }
  return NULL;
}

struct xml_attr
{
  char *name;
  char *value;
  struct xml_node *parent;
  SLIST_ENTRY(xml_attr) entry;
};

struct xml_node
{
  char *tag;
  struct xml_node *parent;
  SLIST_HEAD(,xml_attr) attrs;
  SLIST_HEAD(,xml_node) children;
  SLIST_ENTRY(xml_node) sibling;
};

/* latitude   -90..0..90 = 69.407 .. 68.703 .. 69.407 */
/* longitude  -180 .. 0 .. 180 =  */

#define SIZE 1000
int main(int argc, char *argv[])
{
  struct stat st;
  int fd;
  void *ptr, *np, *npe, *s;
  uint64_t id;
  double lat,lon,minx,miny,maxx,maxy,sz;
  struct node *node, *lc, *cc;
  matrix_t m;
  vector_t v0,v1;
  bitmap_t *bm;
  minx = 180;
  miny = 90;
  maxx = -180;
  maxy = -90;

  SLIST_INIT(&nlist);
  if ((fd = open(argv[1], O_RDONLY)) < 0)
    return -1;
  if (fstat(fd, &st) < 0)
    return -1;
  ptr = mmap(0, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
  if (ptr == (void *)-1LL)
    return -1;

  printf("Got mmap : %p %lx\n", ptr, st.st_size);
  np = ptr;
  while ((np = strstr(np, "<node")) != NULL) {
    sscanf(np,"<node id=\"%lld\" lat=\"%lf\" lon=\"%lf\"", &id, &lat, &lon);

    node = malloc(sizeof(*node));
    node->id = id;
    node->lat = lat;
    node->lon = lon;
    if (lon < minx)
      minx = lon;
    if (lon > maxx)
      maxx = lon;

    if (lat < miny)
      miny = lat;
    if (lat > maxy)
      maxy = lat;
    SLIST_INSERT_HEAD(&nlist, node, entry);
    np++;
  }
  //minx = -97.753;
  //maxx = -97.72732;
  //miny = 30.26099;
  //maxy = 30.27701;
  sz = (maxx-minx);
  if (sz < (maxy-miny))
    sz = maxy-miny;
  printf("Box = [%lf,%lf] - [%lf,%lf]\n", minx,miny,maxx,maxy);
  
  bm = bmp_alloc(SIZE,SIZE);
  m = matxmat(S(SIZE/sz,SIZE/sz),T(-minx,-miny));

  np = ptr;
  while ((np = strstr(np, "<way")) != NULL) {
    printf("----\n");
    lc = NULL;
    if ((npe = strstr(np, "</way")) != NULL) {
      for (s=np; s != npe; s++) {
	if (sscanf(s, "<nd ref=\"%lld\"", &id) == 1) {
	  cc = findnode(id);
	  if (lc != NULL && cc != NULL) {
	    v0 = matxvec(m, vecinit(lc->lon, lc->lat));
	    v1 = matxvec(m, vecinit(cc->lon, cc->lat));
	    bmp_line(bm, v0.v[0], v0.v[1], v1.v[0], v1.v[1], RGB(0,0xFF,0));
	  }
	  lc = cc;
	  printf("nd = %lld\n", id);
	}
      }
    }
    np = npe;
  }
  bmp_write(bm,"ausmap.bmp");
  return 0;
}
