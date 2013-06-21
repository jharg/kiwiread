#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "bmp.h"

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

void bmp_free(bitmap_t *bmp)
{
  free(bmp->data);
  free(bmp);
}

int bmain()
{
  bitmap_t *b;
  int i,j;

  b = bmp_alloc(128,128);
  for (i=0; i<128; i++) {
    bmp_putpixel(b, i, 10, RGB(0xFF,0,0));
    bmp_putpixel(b, i, 20, RGB(0,0xFF,0));
    bmp_putpixel(b, i, 30, RGB(0,0,0xFF));
  }
  bmp_write(b, "x.bmp");
}
