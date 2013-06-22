#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "bmp.h"
#include <math.h>

static void *zmalloc(size_t sz)
{
  void *ptr;

  if ((ptr = malloc(sz)) != NULL)
    memset(ptr, 0, sz);
  return ptr;
}

bitmap_t *bmp_alloc(int w, int h)
{
  bitmap_t *bmp;

  w = (w + 3) & ~3;
  bmp = zmalloc(sizeof(*bmp));
  bmp->data = zmalloc(w*h*sizeof(int));
  bmp->w = w;
  bmp->h = h;
}

void bmp_putpixel(bitmap_t *bmp, int x, int y, int rgb)
{
  if (x >= bmp->w || y >= bmp->h)
    return;
  bmp->data[x+y*bmp->w] = rgb;
}

void bmp_hline(bitmap_t *bmp, int x0, int x1, int y0, int rgb)
{
  int tmp;
  if (x0 > x1) {
    tmp = x0;
    x0 = x1;
    x1 = tmp;
  }
  while (x0 <= x1)
    bmp_putpixel(bmp, x0++, y0, rgb);
}

void bmp_vline(bitmap_t *bmp, int x0, int y0, int y1, int rgb)
{
  int tmp;

  if (y0 > y1) {
    tmp = y0;
    y0 = y1;
    y1 = tmp;
  }
  while (y0 <= y1)
    bmp_putpixel(bmp, x0, y0++, rgb);
}

void bmp_line(bitmap_t *bmp, int x0, int y0, int x1, int y1, int rgb)
{
  int dx, dy, sx, sy, e, e2;

  if (x0 == x1)
    bmp_vline(bmp, x0, y0, y1, rgb);
  else if (y0 == y1)
    bmp_hline(bmp, x0, x1, y0, rgb);
  else {
    sx = sy = 1;

    if ((dx = x1 - x0) < 0) {
      sx = -1;
      dx = -dx;
    }
    if ((dy = y1 - y0) < 0) {
      sy = -1;
      dy = -dy;
    }
    e = dx - dy;

    for(;;) {
      bmp_putpixel(bmp, x0, y0, rgb);
      if (x0 == x1 && y0 == y1)
	break;
      e2 = 2*e;
      if (e2 > -dy) {
	e -= dy;
	x0 += sx;
      }
      if (x0 == x1 && y0 == y1) {
	bmp_putpixel(bmp, x0, y0, rgb);
	break;
      }
      if (e2 < dx) {
	e += dx;
	y0 += sy;
      }
    }
  }
}

int vec[] = {
   55,27, /* Ascii 64 */
   18,13,17,15,15,16,12,16,10,15, 9,14, 8,11, 8, 8, 9, 6,11, 5,14, 5,16,
    6,17, 8,-1,-1,12,16,10,14, 9,11, 9, 8,10, 6,11, 5,-1,-1,18,16,17, 8,
   17, 6,19, 5,21, 5,23, 7,24,10,24,12,23,15,22,17,20,19,18,20,15,21,12,
   21, 9,20, 7,19, 5,17, 4,15, 3,12, 3, 9, 4, 6, 5, 4, 7, 2, 9, 1,12, 0,
   15, 0,18, 1,20, 2,21, 3,-1,-1,19,16,18, 8,18, 6,19, 5,
};

int vec2[] = {
  8,18, /* Ascii 65 */
  9,21, 1, 0,-1,-1, 9,21,17, 0,-1,-1, 4, 7,14, 7,-1,-1,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
};

static void _put32(uint8_t *p, int v)
{
  p[0] = v;
  p[1] = v >> 8;
  p[2] = v >> 16;
  p[3] = v >> 24;
}

static void _put16(uint8_t *p, uint16_t v)
{
  p[0] = v;
  p[1] = v>>8;
}

#define HDRLEN 54
void bmp_write(bitmap_t *bmp, const char *file)
{
  FILE *fp;
  int len, i, j;
  uint8_t h[HDRLEN] = { 0 };

  if ((fp = fopen(file, "w")) != NULL) {
    len = HDRLEN + 3*bmp->w*bmp->h;

    h[0] = 'B';
    h[1] = 'M';
    _put32(h+2, len);
    _put32(h+0xA, HDRLEN);           // offset of bmp data

    _put32(h+0xe, 40);               // size of DIB header
    _put32(h+0x12, bmp->w);          // width
    _put32(h+0x16, bmp->h);          // height
    _put16(h+0x1a, 1);               // planes
    _put16(h+0x1c, 24);              // bits per pixel
    _put32(h+0x22, 16);
    _put32(h+0x26, 2835);
    _put32(h+0x2a, 2835);

    h[34] = 16;
    h[36] = 0x13;
    h[37] = 0x0b;
    h[42] = 0x13;
    h[43] = 0x0b;

    fwrite(h, HDRLEN, 1, fp);

    for (j=0; j<bmp->h; j++) {
      for (i=0; i<bmp->w; i++) {
	uint8_t rgb[3];
	int pixel;

	pixel = bmp->data[i + j*bmp->w];
	rgb[0] = pixel;
	rgb[1] = pixel >> 8;
	rgb[2] = pixel >> 16;
	fwrite(rgb, 3, 1, fp);
      }
    }
    fclose(fp);
  }
}

/* T(px,py,1)*S(sx,sy,1)*T(-px,-py,1) */

/* 0      0       tx
 * 0      0       ty
 * 0      0       1
 *
 * sx     0       0
 * 0      sy      0
 * 0	  0       1
 *
 * cos(a) -sin(a) 0
 * sin(a) cos(a)  0
 * 0      0       1
 */
struct vector
{
  double v[3];
};

struct matrix
{
  double m[3][3];
};

struct vector matxvec(struct matrix m, struct vector v)
{
  struct vector r;
  double  normal;

  normal = ((m.m[2][0]*v.v[0]) + (m.m[2][1]*v.v[1]) + (m.m[2][2]*v.v[2])); 
  r.v[0]   = ((m.m[0][0]*v.v[0]) + (m.m[0][1]*v.v[1]) + (m.m[0][2]*v.v[2])) / normal;
  r.v[1]   = ((m.m[1][0]*v.v[0]) + (m.m[1][1]*v.v[1]) + (m.m[1][2]*v.v[2])) / normal;
  r.v[2]   = 1;
  return r;
}

struct matrix matxmat(struct matrix a, struct matrix b)
{
  int i, j, k;
  struct matrix r = { 0 };

  for (i=2;i>=0;i--) {
    for (j=2;j>=0;j--) {
      for (k=2;k>=0;k--) {
	r.m[i][j] += a.m[i][k]*b.m[k][j];
      }
      r.m[i][j] /= r.m[2][2];
    }
  }
  return r;
}

struct matrix I()
{
  struct matrix r = { 0 };
  r.m[0][0] = r.m[1][1] = r.m[2][2] = 1;
  return r;
}

struct matrix T(int tx, int ty)
{
  struct matrix r = { 0 };

  r.m[0][0] = 1;
  r.m[0][2] = tx;
  r.m[1][1] = 1;
  r.m[1][2] = ty;
  r.m[2][2] = 1;
  return r;
}

struct matrix S(int sx, int sy)
{
  struct matrix r = { 0  };
  
  r.m[0][0] = sx;
  r.m[1][1] = sy;
  r.m[2][2] = 1;
  return r;
}

struct matrix R(int angle)
{
  double rad = (angle * M_PI);
  struct matrix r = { 0 };
  
  rad /= 180.0;
  r.m[0][0] = cos(rad);
  r.m[0][1] = -sin(rad);
  r.m[1][0] = sin(rad);
  r.m[1][1] = cos(rad);
  r.m[2][2] = 1;
  return r;
}

void printmat(struct matrix m)
{
  int i, j;

  printf("mtx:\n");
  for (i=0; i<3; i++) {
    for (j=0; j<3; j++) {
      printf("%g ", m.m[i][j]);
    }
    printf("\n");
  }
}

void bmp_drawstring(bitmap_t *bmp, int x, int y, int halign, int valign, int rotate, const char *str)
{
  int tx, ty, sx, sy, len;
  
  sx = 1;
  sy = 1;
  len = strlen(str);
  if (halign == CENTER)
    x = -(len * 8)/2;
  else if (halign == RIGHT)
    x = -len*8;
  if (valign == TOP)
    y = -8;
}

void bmp_free(bitmap_t *bmp)
{
  free(bmp->data);
  free(bmp);
}

void bmp_drawvec(bitmap_t *bmp, int x, int y, int *bvec, int rgb)
{
  int pos, n = bvec[0];
  int i, x0, x1, y0, y1;
  struct vector p0,p1,v0,v1;
  struct matrix m;
  int halign,valign;

  /* x1 = y1 = -1 */
  v1.v[0] = -1;
  v1.v[1] = -1;
  v1.v[2] = 1;

  /* valign == 0 || 22 */
  /* halign == 0 || -bvec[1]/2 || bvec[1] */
  halign = -bvec[1]; 
  valign = 0;

  pos = 2;
  m = matxmat(T(x,y),R(0));
  m = matxmat(m,T(halign,valign));
  for (i=0; i<n; i++) {
    v0 = v1;
    v1.v[0] = bvec[pos++];
    v1.v[1] = bvec[pos++];
    v1.v[2] = 1;
    if (v0.v[0] != -1 && v1.v[0] != -1) {
      p0 = matxvec(m,v0);
      p1 = matxvec(m,v1);
      bmp_line(bmp, p0.v[0], p0.v[1], p1.v[0], p1.v[1], rgb);
    }
  }
}

/* L = T(tx,ty)*S(sx,sy)*R(a)*T(-tx,-ty) */
int main()
{
  bitmap_t *b;
  int i,j;
  struct matrix t,s,r;

  b = bmp_alloc(128,128);
  bmp_drawvec(b, 50, 50, vec2, RGB(0xFF,0xFF,0));
  bmp_line(b, 0, 50, 128, 50, RGB(0,0,0xFF));
  bmp_line(b, 50, 0, 50, 128, RGB(0,0xFF,0));
  bmp_write(b, "x.bmp");
}
