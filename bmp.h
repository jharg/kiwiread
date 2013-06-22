typedef struct 
{
  int w,h;
  int *data;
} bitmap_t;

bitmap_t *bmp_alloc(int w, int h);
void bmp_putpixel(bitmap_t *bmp, int x, int y, int rgb);
void bmp_write(bitmap_t *bmp, const char *file);
void bmp_free(bitmap_t *bmp);

#define RGB(r,g,b) ((b) + ((g) << 8) + ((r) << 16))

enum halign_t {
  LEFT,
  CENTER,
  RIGHT
};

enum valign_t {
  TOP,
  BOTTOM
};
