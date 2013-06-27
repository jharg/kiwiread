#ifndef __bmp_h__
#define __bmp_h__
typedef struct
{
  double v[3];
} vector_t;

typedef struct
{
  double m[3][3];
} matrix_t;

vector_t vecinit(double, double);
vector_t matxvec(matrix_t, vector_t);
matrix_t matxmat(matrix_t, matrix_t);
matrix_t I();
matrix_t T(double tx, double ty);
matrix_t S(double sx, double sy);
matrix_t R(double angle);
void printmat(matrix_t m);

typedef struct 
{
  int w,h;
  int *data;
} bitmap_t;

bitmap_t *bmp_alloc(int w, int h);
void bmp_putpixel(bitmap_t *bmp, int x, int y, int rgb);
void bmp_write(bitmap_t *bmp, const char *file);
void bmp_free(bitmap_t *bmp);
void bmp_drawstring(bitmap_t *bmp, int x, int y, int halign, int valign, int ang, const char *str, int rgb);

void bmp_rect(bitmap_t *bmp, int x0, int y0, int x1, int y1, int rgb);
void bmp_line(bitmap_t *bmp, int x0, int y0, int x1, int y1, int rgb);
void bmp_poly(bitmap_t *bmp, int nvertex, int *xy, int rgb);
void bmp_polyline(bitmap_t *bmp, int nvertex, int *xy, int rgb);

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

enum orient_t {
  HORIZ,
  VERT
};
#endif
