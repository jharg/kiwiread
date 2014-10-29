/*
    Copyright (C) 2013 Jordan Hargrave<jordan_hargrave@hotmail.com>

    kiwiread.c - main code for reading ALLDATA.KWI Car Nav DVD

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
#define _LARGEFILE64_SOURCE
#define _LARGEFILE_SOURCE
#define _FILE_OFFSET_BITS 64
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <inttypes.h>
#include <time.h>
#include <endian.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <assert.h>
//#include <oslib.h>
#include "bmp.h"

#define _PACKED __attribute__((packed))
#pragma pack(1)

int isin(double y, double x, double ly, double lx, double dy, double dx);

typedef struct quadtree
{
  int left,bottom,top,right;
  struct quadtree *nodes[4];
} quadtree_t;

// 00 = ll, 01 = lr, 10 = ul, 11 = ur
void quadtree_insert(quadtree_t **proot, int x, int y)
{
  int cx, cy, idx = 0;
  quadtree_t *root = *proot;
  
  if (x < root->left || x > root->right ||
      y < root->bottom || y > root->top)
    return;
  cx = root->left + (root->right - root->left)/2;
  cy = root->bottom + (root->top - root->bottom)/2;

  if (x > cx) 
    idx |= 0x1;
  if (y >= cy)
    idx |= 0x2;
  quadtree_insert(&root->nodes[idx], x, y);
} 

struct key_t
{
  int key;
  const char *value;
  const char *osm_key;
  const char *osm_value;
};

const char *lookup(struct key_t *tab, int val)
{
  while (tab->key != -1) {
    if (tab->key == val)
      return tab->value;
    tab++;
  }
  printf("unknown: %x\n", val);
  return "";
}

int rdtype[65536];

void showrt(int *rt)
{
  int i;

  for (i=0; i<65536;  i++)
    if (rt[i])
      printf("%5x: %d\n", i, rt[i]);
}

struct key_t types[] = {
  { 0x101, "unknown 101" },
  { 0x102, "unknown 102" },
  { 0x103, "unknown 103" },
  { 0x104, "unknown 104" },
  { 0x105, "unknown 105" },
  { 0x106, "unknown 106" },

  { 0x121, "water system (shore line,ocean,bay,sea,creek)" },
  { 0x122, "water system (lake,marsh,pond)" },
  { 0x123, "water system (river)" },
  { 0x124, "water system (canal, irrigation canal)" },
  { 0x128, "island" },
  { 0x131, "address level 1 (country)" },
  { 0x132, "address level 2 (state)" },
  { 0x134, "address level 4 (municipality)" },
  { 0x140, "urban district" },
  { 0x141, "green belt, park" },
  { 0x142, "factory, factory site" },
  { 0x1FE, "information highway symbol" },

  { 0x201, "other transport 1" },

  { 0x210, "road type 0" },
  { 0x211, "road type 1" },
  { 0x212, "road type 2" },
  { 0x213, "road type 3" },
  { 0x214, "road type 4" },
  { 0x215, "road type 5" },
  { 0x216, "road type 6" },
  { 0x217, "road type 7" },
  { 0x218, "road type 8" },
  { 0x21b, "road type 11" },
  { 0x21c, "road type 12" },
  { 0x21d, "road type 13" },
  { 0x242, "very high speed railway, JR line [main line]" },
  { 0x280, "other airport" },
  { 0x408, "cemetery" },
  { 0x43c, "national defense facility, base" },
  { 0x464, "university, college" },
  { 0x480, "hospital" },
  { 0x520, "other sports facility" },
  { 0x566, "historic spot, scenic spot, natural monument"  },
  { 0x620, "other shopping facility" },
  { 0x6180, "golf course" },
  { -1 },
};

/* Decode 1,2,3,4 BigEndian bytes */
uint8_t _1b(const void *v)
{
  return *(uint8_t *)v;
}

uint16_t _2b(const void *v)
{
  const uint8_t *s = v;

  return ((s[0] << 8) + s[1]);
}

uint32_t _3b(const void *v)
{
  const uint8_t  *s = v;

  return ((s[0] << 16) + (s[1] << 8) + s[2]);
}

uint32_t _4b(const void *v)
{
  const uint8_t *s = v;

  return ((s[0] << 24) + (s[1] << 16) + (s[2] << 8) + s[3]);
}

int D(int v) 
{
  if (v != 0xFFFF) 
    v <<= 1;
  return v;
}

int SWS(int v)
{
  if (v != 0xFFFF)
    v <<= 1;
  return v;
}

FILE *df;

static void os_dump(void *b, int len)
{
  unsigned char *bb = (unsigned char *)b;
  int i,j,c;

  if (df == NULL)
    df = stdout;
  for(i=0;i<len;i+=16) {
    fprintf(df,"%.4x: ", i);
    for(j=0;j<16;j++) {
      if (i+j >= len)
	fprintf(df,"XX ");
      else
	fprintf(df,"%.02x ", bb[i+j]);
    }
    fprintf(df,"  ");
    for(j=0;j<16;j++) {
      c = bb[i+j];
      if (i+j >= len || c < ' ' || c > 'z')
	c = '.';
      fprintf(df,"%c",c);
    }
    fprintf(df,"\n");
  }
}

/*
 * N = Natural Number 0..maxint*2
 * I = Integeger -maxint .. maxint
 * C = 1 byte string
 * CC = meta string
 * B = bit
 * NZ = 0.0 <= n <= 1.0
 * SA = sector address
 */

int sector_sz = 2048;
int logical_sz = 32;

/* 1.2.7 ssssssss ssssssss ssssssss DLnnnnnn */
typedef uint32_t sectoraddress_t;

/* Fsssssss ssssssss sssssfff */
#define LATITUDE   "NS"
#define LONGITUDE  "EW"
#define LL_DIR     (1L << 23)

typedef uint8_t geonum_t[3];

double _lx,_ly,_rx,_ry, _mx, _my;

double geo_secs(geonum_t geo)
{
  int32_t v,flag,secs;

  v = _3b(geo);
  flag = !!(v & LL_DIR);
  secs = v & ~LL_DIR;
  if (flag)
    secs = -secs;
  return secs / (3600.0 * 8);
}

void printgeo(geonum_t geo, const char *ll)
{
  uint32_t v,flag,secs;

  v = _3b(geo);
  flag = !!(v & LL_DIR);
  secs = v & ~LL_DIR;
  printf("%c%lf", ll[flag], secs / (3600.0 * 8));
#if 0
  printf("%c%d %d %lf [%lf]", 
	 ll[flag], 
	 secs / (3600 * 8), 
	 ((secs / ( 60 * 8)) % 60),
	 (secs % (60 * 8)) / 8.0,
	 geo_secs(geo));
#endif
}

void printlatlng(geonum_t lat, geonum_t lng)
{
  printgeo(lat, LATITUDE);
  printf(" ");
  printgeo(lng, LONGITUDE);
}

/* 1.2.13 Parcel ID */
struct pid_t
{
  geonum_t      lat;
  uint8_t       exp1;
  geonum_t      lng;
  uint8_t       exp2;
}  _PACKED;

/* 1.2.14 Manufacturer ID */
struct mid_t
{
  struct pid_t	loc;
  uint8_t	floor;
  uint8_t	reserved;
  uint16_t	date;
} _PACKED;

struct lmm_t
{
  uint16_t 	n;
  uint16_t 	dummy;
} _PACKED;

struct sii_t
{
  struct mid_t 	mid;
  uint16_t 	nm;
  uint16_t 	dummy;
} _PACKED;

struct mii_t
{
  uint8_t 		category;
  uint8_t 		reserved[3];
  uint8_t 		modulename[52];
  uint8_t 		modulever[8];
} _PACKED;

struct mmi_t 	
{
  uint16_t 		date_start;
  uint16_t 		date_end;
  uint8_t  		title[64];
  uint8_t  		mandep[182];
  uint32_t 		addr;
  uint16_t 		size;
} _PACKED; 

/* LOADING.KWI
 * mii_t {
 *   uint8_t category;
 *   uint8_t reserved[3];
 *   uint8_t modulename[52];
 *   uint8_t modulever[8];
 * };
 * mmi_t {
 *   uint16_t date_start;
 *   uint16_t date_end;
 *   uint8_t  title[64];
 *   uint8_t  mandep[182];
 *   u32 addr;
 *   uint16_t size;
 * }; 
 * smi_t {
 *   mii_t[mn];
 *   mmi_t[mn];
 * };
 * sii_t {
 *    uint8_t  sid[12];
 *    uint16_t mn;
 *    uint16_t reserved;
 * };
 * lmm_t {
 *    uint16_t  n;
 *    uint16_t  reserved;
 *    struct 	sii_t[n];
 *    struct	smi_t[n]; { mii_t[mn], mmi_t[mn] };
 * }
 */

/* ALLDATA.KWI
 * datavolume
 * [n] management header tables
 */

/* 5.1 Data Volume */
struct datavol_t
{
  struct mid_t	spec_mid;           // 00  [MID]
  char		spec_ssi[52];       //     [C.52]
  struct mid_t	data_mid;           // 64  [MID]
  char		data_ssi[52];       //     [C.52]
  struct mid_t	system_mid;         // 128 [MID]
  char		system_ssi[20];     //     [C.20]

  char		format_ver[64];     // 160 [C.64]
  char		data_ver[64];       // 224 [C.64]
  char		disk_title[128];    // 288 [C.128]
  uint16_t      contents[4];        // 416 [B.8]
  char		media_version[32];  // 424 [C.32]

  /* Data coverage */
  struct pid_t	box_ll;             // 456 Data Coverage
  struct pid_t	box_ur;

  uint16_t	log_size;		// logical sector size
  uint16_t	sector_size;		// sector size
  uint16_t	background;		// [B:B] background data default information

  uint8_t	res1[14];
  uint8_t	level_mgmt[256];	// level management information
  uint8_t	res2[1300];
} _PACKED;

off_t getsector(sectoraddress_t sa)
{
  off_t na;

  na = (sa >> 8) * sector_sz + (sa & 0x3F) * logical_sz;
#if 0
  printf("GetSector: %c%c%d.%d\n",
	 sa & 0x80 ? 'B' : 'A',
	 sa & 0x40 ? 'D' : 'S',
	 sa  >> 8,
	 sa & 0x3F);
#endif
  return na;
}

void showdate(uint16_t tstamp)
{
  struct tm tm;
  time_t base;
	
  memset(&tm, 0, sizeof(tm));

  /* Start date = Jan 1, 2007 */
  tm.tm_year = 97;
  tm.tm_mday = 1;
  tm.tm_mon = 0;
  base = mktime(&tm) + (60 * 60 * 24) * tstamp;
  printf("%s", ctime(&base));
}

void showpid(struct pid_t pid)
{
  printlatlng(pid.lat, pid.lng);
}

void showmid(struct mid_t mid)
{
  printf(" date: ");
  showdate(mid.date);
  printf(" ");
  showpid(mid.loc);
  printf("  floor:%d\n", mid.floor);
}

void *zmalloc(size_t size)
{
  void *ptr;

  if ((ptr = malloc(size)) != NULL)
    memset(ptr, 0, size);
  return ptr;
}

size_t nm;

void zfree(void *p, size_t sz)
{
  nm -= sz;
  free(p);
}

off_t fend;
bitmap_t *fext;
int nfext;

void *zread(int fd, int sz)
{
  void *ptr;
  off_t off;

  ptr = zmalloc(sz);
  off = lseek(fd, 0, SEEK_CUR);
  read(fd, ptr, sz);
  nfext++;

  return ptr;
}

void *zreado(int fd, size_t sz, off_t off, char *lbl)
{
  void *ptr;

  nm += sz;
  ptr = zmalloc(sz);
  pread(fd, ptr, sz, off);
  nfext++;
  return ptr;	
}

const char *mii_category[] = { "initial program", "program", "library", "data" };

void swapw(uint16_t *w)
{
  *w = _2b(w);
}

void swapl(uint32_t *l)
{
  *l = _4b(l);
}

struct datadir_entry
{
  uint32_t magic;
  uint32_t reserved;
  uint32_t offset;
  uint32_t length;
};

struct datadir
{
  uint16_t count;
  uint16_t reserved;
};

/* Dump manufacturer dependent portion */
void dump_mandep(int fd, void *buffer)
{
  struct datadir *dd;
  struct datadir_entry *dde;
  char key[5];
  void *mbuf;
  int i;

  dd = buffer;
  dde = (void *)&dd[1];
  swapw(&dd->count);

  memset(key, 0, sizeof(key));
  for (i=0; i<dd->count; i++) {
    swapl(&dde[i].offset);
    swapl(&dde[i].length);
    memcpy(key, &dde[i].magic, 4);
    printf("magic: %4s  off: %.8" PRIx32 " end: %.8" PRIx32 "\n", key, dde[i].offset * 2 + 0x800, 
	   dde[i].length + dde[i].offset * 2 + 0x800 - 1);

    mbuf = zreado(fd, dde[i].length, SWS(dde[i].offset) + 0x800, "mandep");
    os_dump(mbuf, dde[i].length);
    zfree(mbuf, dde[i].length);
  }
}

void dumpbm(int fd)
{
  off_t off;
  char buf[2];
  uint32_t size;
  char name[32];
  int fdo;
  uint8_t *zm;

  lseek(fd, 0, SEEK_SET);
  for(;;) {
    off = lseek(fd, 0, SEEK_CUR);
    snprintf(name, sizeof(name), "bm%x.bmp", off);
    if (read(fd, buf, 2) != 2)
      break;

    if (!memcmp(buf, "BM", 2)) {
      read(fd, &size, 4);
      if (size > 0x200000) {
	lseek(fd, off+2, SEEK_SET);
	continue;
      }
      printf("size = %" PRIx32 "\n", size);
      lseek(fd, off, SEEK_SET);
      zm = zread(fd, size+100);
      fdo = open(name, O_RDWR|O_CREAT, 0660);
      write(fdo, zm, size+100);
      close(fdo);

      lseek(fd, off+6, SEEK_SET);
      off += 4;
    }
    off += 2;
  }		
}

struct mhr_t
{
  sectoraddress_t 	dsa;
  uint16_t		size;
  char			name[12];
} _PACKED;

/* 6.1 Parcel Data Management Distribution Header */
struct pdmdh_t
{
  uint16_t		size;          // [SWS]
  uint16_t		res1;
  uint16_t		filename;      // [:B]
  uint16_t		res2;

  geonum_t		upper_lat;     // [B:N]
  geonum_t		lower_lat;     // [B:N]
  geonum_t		left_lng;      // [B:N]
  geonum_t		right_lng;     // [B:N]

  uint16_t		lmr_sz;        // [SWS]
  uint16_t		bsmr_sz;       // [SWS]
  uint16_t		bmr_sz;        // [SWS]

  uint16_t		nlmr;         // [N]
  uint16_t		nbsmr;        // [N]

  /* lmr_t        lmr[nlmr];     Level Management Records
   * bsmr_t       bsmr[nbsmr];   Block Set Management Records
   * bmt_t        bmt[var];      Block Management Tables
   */
} _PACKED;

/* 6.1.1 Level Management Record */
typedef struct lmr_t
{
  uint16_t		header;       // [I:N] level management header
  uint16_t		numbers;      // [N:N:N:N] number of basic/ext data frames
  uint32_t		dispflag[5];  // [N] display scale flags
  struct {
    uint8_t             lat;          // N:N # of lat block sets
    uint8_t             lng;          // N:N # of lng block sets
  } _PACKED nblocksets;
  struct {
    uint8_t             lat;          // N:N # of lat blocks
    uint8_t             lng;          // N:N # of lng blocks
  } _PACKED nblocks;
  struct {
    uint8_t             lat;          // N:N # of lat (divided) parcels
    uint8_t             lng;          // N:N # of lng (divided) parcels
  } _PACKED nparcels[4];              // 0 = normal, 1=pardiv1, 2=pardiv2, 3=pardiv3

  uint16_t		bsmr_off;     // [D]   offset from zdat[0]
  uint16_t		nrsize;       // [SWS] node record size
} _PACKED lmr_t;

/* 6.1.2 Block Set Management Record */
struct bsmr_t
{
  uint16_t		header;        // [I:N] header (level,blockset)
  uint32_t		bmt_offset;    // [D]   offset to block management table from zdat[0]
  uint32_t		bmt_size;      // [SWS] size of block management table
} _PACKED;

/* 6.2.1 Block Management Record */
struct bmt_t
{
  sectoraddress_t       dsa;           // [DSA] sector address or reference
  uint16_t              size;          // [BS]  size in logical sectors
} _PACKED;

struct bmtfile_t
{
  sectoraddress_t       dsa;           // [DSA] sector address or reference
  uint16_t              size;          // [BS]  size in logical sectors
  uint8_t               filename[12];  // [C]   optional
};

/* Main Map Parcel Management Record */
typedef union {
  struct {
    sectoraddress_t      dsa;      // [DSA] sector address
    uint16_t             size;     // [BS] size in logical sectors
  } _PACKED map0[1];
  struct {
    sectoraddress_t      dsa;
    uint16_t             size1;
    uint16_t             size2;
  } _PACKED map1[1];
  struct {
    sectoraddress_t      dsa;
    uint16_t             size1;
    uint16_t             size2;
  } _PACKED map2[1];
  struct {
    sectoraddress_t      dsa;
    uint16_t             size;
    char                 filename[12];
  } _PACKED map100[1];
} _PACKED mapinfo_t;

/* Parcel Management Info */
struct parman_t
{
  uint16_t             type;       // [N:N] Parcel Management type
  uint16_t             routeoff;   // [D] route guidance parcel management list
  // mapinfo_t mapdata[];
};

/* Road Data frame
 * [var] Road Distribution Header
 * [var] Road Data List
 * [var] Passage Code Data Frame
 * [var] Composite Node Data Frame
 * [var] Expansion Data
 *
 *
 * Road Distribution Header
 * uint16_t header size;
 * uint16_t number of intersections;
 * uint8_t  number of display classes;
 * uint8_t  count of additional data
 * uint16_t level of route planning data
 * [var]    sequence of display class management
 * [var]    sequence of additional data
 * [var]    expansion data
 *
 *
 */

int signex(uint32_t val, int start, int end)
{
  uint32_t mask = (1L << (end - start + 1)) - 1;
  uint32_t bits = (val >> start) & mask;

  /* If upper bit is set... mask in -1 */
  if (bits & (1L << (end - start)))
    bits |= (-1L & ~mask);
  return bits;
}

static inline uint32_t extract(uint32_t val, int start, int end)
{
  uint32_t mask = (1L << (end - start + 1)) - 1;
  return (val >> start) & mask;
}

struct pid_t ll,ur;

/* x = # of lng blocks
 * y = # of lat blocks
 *
 *   +--+--+--+--+
 *   |  |  |  |  |
 *   +--+--+--+--+
 *   |  |  |  |  |
 *   +--+--+--+--+
 *   |  |  |  |  |
 * ^ +--+--+--+--+
 * | |  |  |  |  |
 * y +--+--+--+--+
 *   x->
 */
struct lmr_t *findlevel(struct lmr_t *map[], int n, int lvl)
{
  int i;

  for (i=0; i<n; i++) {
    if (lvl == extract(map[i]->header, 10, 15))
      return map[i];
  }
  return NULL;
}


int min(int a, int b)
{
  return (a < b) ? a : b;
}

struct mfde_t {
  uint32_t offset;
  uint16_t size;
} _PACKED;

struct mapframe_t {
  uint16_t             size;    // [SWS]
  struct pid_t         llpid;   // [PID]
  uint16_t             llcode;  // [N:N]
  uint16_t             dipid;   // [N:B:N:N:N]
  uint32_t             pmcode;  // [N:B:B:B:B:B]
  uint16_t             dsflag;  // [B:B:N]
  uint16_t             rlx;     // [B:N]
  uint16_t             rly;     // [B:N]
  uint16_t             geo_str; // [I]
  uint16_t             geo_dec; // [I]
  sectoraddress_t      rg_addr; // [DSA]
  uint16_t             rg_size; // [BS]
  uint16_t             nregion; // [N]
  /* [VAR] Regions x4 */
  /* [VAR] mfde_t[] */
} _PACKED;

bitmap_t *bm;
matrix_t  m;

/* Bitmap X/Y size */
int      bmsz = 2048;

void setxy(int *pts, int x, int y)
{
  vector_t v;
  
  v = matxvec(m,vecinit(x,y));
  pts[0] = v.v[0];
  pts[1] = v.v[1];
}

typedef struct
{
  uint16_t               sws;
  uint16_t               ninter;
  uint16_t               ndc;
  uint16_t               naddl;
  uint16_t               lvl;
} _PACKED roadhdr_t;

typedef struct
{
  uint16_t               offset;
  uint16_t               npoly;
} _PACKED dcmr_t;

typedef struct
{
  uint32_t               hdr;
  uint16_t               nodehdr;
  uint16_t               shapehdr;
  uint16_t               addlhdr;
  uint16_t               althdr;
  uint16_t               passhdr;
  uint16_t               attrib;
} _PACKED multilink_t;

/* 7.2.1 Road Distribution Header                       mandatory
 *   00 [SWS]					        mandatory
 *   02 [N] intersections				mandatory
 *   04 [N] # display classes[n]			mandatory
 *   05 [N] # additional data[m]			mandatory
 *   06 [I] level of route planning data		mandatory
 *   08 [n] Display Class Management Records	        opt
 *       10+2x [D] Offset by display class		mandatory
 *       12+2x [B:N] Number of polylines[ri]            mandatory 
 *   [m] Additional Data Management Records	        opt
 * 7.2.2 Road Data List                                 opt [n count]
 *   [B...] Display Scale Flag
 *   [ri]   Multilink Data Record
 * Passage Code Data Frame        opt
 * Composite Node Data Frame      opt
 * Expansion Data                 opt
 */
#define zprintf(x...) 

void dumproad(void *ptr, size_t len)
{
  size_t hlen = SWS(_2b(ptr));
  int ninter, ndc, nad, lvl, i, off, xoff, xsz, npoly, last, j, hdr, noff, nlen, lattr;
  int sx, sy, xc, yc, rx, ry, k, nnodes, nip, l, rtype;
  int pts[20000],npts,clr;

  /* 7.2.1 */
  ninter = _2b(ptr + 2);   // Total Number of Intersections
  ndc = _1b(ptr + 4);      // Number of Display Classes[n]
  nad = _1b(ptr + 5);      // Count of Additional Data[m]
  lvl = extract(_2b(ptr + 6), 10, 15);
  zprintf("========== road len:%.4x inter:%.4x  disp:%2d addl:%d lvl:%d\n", len, ninter, ndc, nad, lvl);

  off = 8;
  
  /* Display Classes */
  last  = 0xFFFF;
  for (i=0; i<ndc; i++) {
    /* 7.2.1.1 Display Class Management Record
     *   00 [D]   Offset by Display Class
     *   02 [B:N]
     *       00:11 Number of polylines by display class [ri]
     */
    xoff = D(_2b(ptr + off));
    npoly = extract(_2b(ptr + off + 2), 0, 11); // Number of Polylines (0..4095)[ri]

    if (xoff != 0xFFFF) {
      zprintf("  [%.4x] npoly:%.4x dispflag:%.4x\n", xoff, npoly, _2b(ptr + xoff));
      xoff += 2;
      for (j=0; j<npoly; j++) {
	/* 7.2.2.1.1.1 MultiLink Header 
	 *    00 [B:B:B:B:SWS:B:SWS] Multilink Management Header
	 *       00:07 [SWS] MultiLink Header Size
	 *          15 [B]   Street Address Info
	 *       16:27 [SWS] MultiLink Size
	 *          28 [B]   Expansion Data
	 *          29 [B]   Expansion Data
	 *          30 [B]   Temporal Information
	 *          31 [B]   MultiLink Delete flag
	 *    04 [N]   Node Information Management Header
	 *       00:10 Number of Nodes
	 *    06 [SWS] Shape Information Header [x]
	 *       00:11 Size of MultiLink Shape Info
	 *    08 [SWS] Additional Node Information Header [y]
	 *       00:11 Size of Additional Information
	 *    10 [SWS] Altitude Information Management Header [z]
	 *       00:11 Size of Altitude Information
	 *    12 [SWS] Passage Information Management Header [w]
	 *       00:11 Size of Passage Regulation Information
	 *
	 *    14 [N:B:B:B:B:B:B:B:N:B:B] MultiLink Attribute
	 *          00 Altitude Flag
	 *          01 Route-type Guidance flag
	 *       02:03 Pseudo 3d Up/Down
	 *          04 Route-planning tag
	 *          06 Link ID
	 *          07 Selected Link Flag
	 *          08 Toll Flag
	 *          09 Route number flag
	 *          10 Infra-link flag
	 *          11 Link ID number flag
	 *       12:15 Road Type code
	 *
	 *    [x] Shape Information
	 *    [x] Link/Node Connection Information
	 *    [y] Additional Node Information
	 *    [z] Altitude Information
	 *    [w] Passage Regulation Information
	 */
	hdr = _4b(ptr + xoff); // multilink management header
	nnodes = extract(_2b(ptr + xoff + 4), 0, 10);
	zprintf("    [%.4x] poly%.4d: nodes:%.4x shape:%.4x additional:%.4x altitude:%.4x passage regulation:%.4x hdr.size:%x size:%x\n", 
	       xoff, j, 
	       nnodes,
	       SWS(extract(_2b(ptr + xoff + 6),0,11)),  // shape
	       SWS(extract(_2b(ptr + xoff + 8),0,11)),  // additional
	       SWS(extract(_2b(ptr + xoff + 10),0,11)), // altitude
	       SWS(extract(_2b(ptr + xoff + 12),0,11)), // passage regulation
	       SWS(extract(hdr, 0,7)),SWS(extract(hdr,16,27)));
	lattr = _2b(ptr + xoff + 14);
	rtype = extract(lattr, 12, 15);
	zprintf("     road type:%d link:%d infra:%d rte:%d toll:%d\n",
	       rtype,
	       extract(lattr, 11, 11),
	       extract(lattr, 10, 10),
	       extract(lattr, 9, 9),
	       extract(lattr, 8, 8));
	if (rtype == 0) {
	  /* highway */
	  bmp_setstyle(bm,"stroke-width:4;");
	  clr = RGB(0,0,0xFF);
	} else if (rtype == 3) {
	  /* main district road */
	  bmp_setstyle(bm,"stroke-width:4;");
	  clr = RGB(0xFF,0x0,0);
	} else if (rtype == 4) {
	  /* prefectural road */
	  bmp_setstyle(bm,"stroke-width:4;");
	  clr = RGB(0xff,0xff,0);
	} else if (rtype == 5) {
	  /* general road 1 (trunk road) */
	  bmp_setstyle(bm,"stroke-width:3;");
	  clr = RGB(0xff,0xff,0);
	} else if (rtype == 6) {
	  bmp_setstyle(bm, "stroke-width:3;");
	  clr = RGB(0xff,0xff,0xff);
	} else if (rtype == 7) {
	  bmp_setstyle(bm, "stroke-width:2;");
	  clr = RGB(54,127,191);
	}

	/* Get offset+size of Shape Data */
	noff = SWS(extract(hdr,0,7));
	nlen = SWS(extract(_2b(ptr + xoff + 6), 0, 11));

	npts = 0;
	for (k=0; k<nnodes; k++) {
	  /* link attribute */
	  lattr = _2b(ptr + xoff + noff);
	  nip = extract(lattr, 0, 9);
	  zprintf("      oneway:%d.%d planned:%d tunnel:%d bridge:%d ip:%d\n",
		 extract(lattr, 15, 15),
		 extract(lattr, 13, 14),
		 extract(lattr, 12, 12),
		 extract(lattr, 11, 11),
		 extract(lattr, 10, 10),
		 nip);

	  /* Get first point */
	  sx = _2b(ptr + xoff + noff + 2);
	  sy = _2b(ptr + xoff + noff + 4);
	  rx = extract(sx, 13, 15);
	  ry = extract(sy, 13, 15);
	  xc = extract(sx, 0, 12) + rx*4096;
	  yc = extract(sy, 0, 12) + ry*4096;

	  setxy(pts+npts, xc, yc);
	  noff += 6;
	  npts += 2;
	  for (l=0; l<nip; l++) {
	    /* Get intermediate points */
	    xc += (char)_1b(ptr + xoff + noff);
	    yc += (char)_1b(ptr + xoff + noff+1);
	    setxy(pts+npts,xc, yc);
	    noff += 2;
	    npts += 2;
	  }
	}
	xoff += SWS(extract(hdr,16,27));
	bmp_polyline(bm, npts/2, pts, clr);
	bmp_setstyle(bm, NULL);
      }
      zprintf("-- end %.4x\n", xoff);
    }
    off += 4;
  }
  /* Additional Data */
  for (i=0; i<nad; i++) {
    xoff = D(_2b(ptr + off));
    xsz  = SWS(_2b(ptr + off + 2));
    off += 4;
    if (xoff != 0xFFFF)
      zprintf("  additional:%.4x  size:%d\n", xoff, xsz);
  }
  zprintf("=========== @roadend\n");
}

/* 7.4 Name Data Frame */
void dumpname(void *ptr, size_t len)
{
  size_t hlen = SWS(_2b(ptr)); // 7.4.1 [SWS] Name Distribution Header
  int na, attr1, attr2,ab,xc,yc,st,off,toff,tnum,k,ang,sx,sy,rx,ry,ds,i;
  char slen, str[256] = { 0 };
  int pts[2];

  zprintf("==================== name\n");
  ab = 0;
  for (off=2; off<hlen; off+=4) {
    /* 7.4.1 Name Data Management Information
     *   00 [D] Offset to Name Data List
     *   02 [N] Number of Name Data Records
     */
    toff = D(_2b(ptr + off));
    tnum = _2b(ptr + off + 2);
    if (toff != 0xFFFF) {
      zprintf("  %2d offset:%.4x  count:%.4x\n", (off-2)/4, toff, tnum);
      for (k=0; k<tnum; k++) {
	/* 7.4.2.1 Name Data Record */

	/* 7.4.2.1.1 Name Attribute Header 
	 *   00 [B:B:B:SWS] Name Data Header
	 *     00:11 Size of Minimum Graphics Record
	 *     13    Extended Data Flag
	 *   02 [B:B:B:B:B:B:B:B:N] Attribute 1
	 *     00:05 Character Priority
	 *     06    String Orientation Flag
	 *     07    Height Information Flag
	 *     08:10 String Type
	 *     11:15 Display Scale Flag X
	 *   04 [N] Attribute 2
	 */
	na    = _2b(ptr + toff);
	attr1 = _2b(ptr + toff + 2);
	attr2 = _2b(ptr + toff + 4);

	ds = extract(attr1, 11, 15);

	st = extract(attr1, 8, 10);    // [B] String type
	zprintf("name: gr:%d str:%d attr1:%x ds:%2x prio:%x type:%x [%s] dir:%s ", 
	       extract(na, 0, 11), st, 
	       attr1, extract(attr1, 11, 15), extract(attr1, 0, 5),
	       attr2, lookup(types, attr2),
	       extract(attr1, 6, 6) ? "vert" : "horz");
	toff += 6;

	/* Get X & Y coordinates */
	if (st != 4) {
	  sx = _2b(ptr + toff + 2);
	  sy = _2b(ptr + toff + 4);
	  
	  rx = extract(sx, 13, 15);
	  ry = extract(sy, 13, 15);
	  xc = extract(sx, 0, 12) + rx*4096;
	  yc = extract(sy, 0, 12) + ry*4096;
	  setxy(pts, xc, yc);
	}

	memset(str, 0, sizeof(str));
	if (st == 1) {
	  /* 7.4.2.1.2  Barycentric string
	   *   00 [B:B:B] Additional Background Info
	   *     11    Auxiliary Data Flag
	   *     13    Additional Background Info Flag
	   *     14:15 Additional Background Info Type
	   *   02 [N:NX]  X Coordinate
	   *     00:12 X-coordinate
	   *     13:15 Relative position within integrated parcel
	   *   04 [N:NX]  Y Coordinate
	   *     00:12 Y-coordinate
	   *     13:15 Relative position within integrated parcel
	   *   06 Character Information Data List
	   *   xx Altitude Information
	   */
	  slen = _2b(ptr + toff + 6)*2;
	  memcpy(str, ptr + toff + 8, slen);
	  zprintf("  string: '%s' x:%d y:%d\n", str, xc, yc);

	  bmp_drawstring(bm, pts[0], pts[1], CENTER, CENTER, 0, 0, str, RGB(0xff,0xFF,0xff));
	  toff += slen + 8; // string length
	} else if (st == 4) {
	  int sp;
	  int npp;

	  /* 7.4.2.1.5 Linear-B 
	  *    00 [B:B:B:B:B:B:N] String Placement Information
	  *    02 [D] Offset to Data
	  *    04 Sequence of String Placement Records 
	  *    xx String Information Data List
	  */
	  xc = _2b(ptr + toff);
	  yc = _2b(ptr + toff + 2);
	  toff += 4;
	  for (i=0; i<extract(xc, 0, 3); i++) {
	    yc = _2b(ptr + toff);
	    zprintf("  orient:%d dist:%d ",
		   extract(yc, 14, 15),
		   extract(yc, 0, 13));
	    toff += 2;
	  }
	  slen = _2b(ptr + toff)*2;
	  memcpy(str, ptr + toff + 2, slen);
	  zprintf(" string:'%s' bit:%d bif:%d aux:%d pt:%d id:%d height:%d #pts:%d\n",
		 str, extract(xc, 14, 15),
		 extract(xc, 13, 13),
		 extract(xc, 11, 11),
		 extract(xc, 8, 10),
		 extract(xc, 6, 7),
		 extract(xc, 5, 5),
		 extract(xc, 0, 3));
	  toff += slen + 2;
	} else if (st == 5) {
	  /* 7.4.2.1.6 Linear-C
	   *   00 [B:B:B] Additional Background Info
	   *     11    Auxiliary Data Flag
	   *     13    Additional Background Info Flag
	   *     14:15 Additional Background Info Type
	   *   02 [N:NX]  X Coordinate
	   *     00:12 X-coordinate
	   *     13:15 Relative position within integrated parcel
	   *   04 [N:NX]  Y Coordinate
	   *     00:12 Y-coordinate
	   *     13:15 Relative position within integrated parcel
	   *   06 [B:B:B:N] Display Angle
	   *   08 Character Information  Data List
	   *   xx Altitude Information
	   */
	  int h,v,o,a;

	  /* Vertical to display angle == horizontal
	   * Parallel to display angle == vertical
	   */
	  ang = _2b(ptr + toff + 6);
	  slen = _2b(ptr + toff + 8)*2;
	  memcpy(str, ptr + toff + 10, slen);
	  if (extract(ang, 10, 11) != 1) {
	    printf("string: '%s' x:%d y:%d angle:%d {%s} orient:%d mode:%d\n", str, xc, yc, 
		   extract(ang, 0, 8),
		   extract(ang, 12,12) ? "rel" : "abs",
		   extract(ang, 10,11),
		   extract(ang, 9, 9));
	  }
	  a = extract(ang, 0, 8) - 90;
	  h = CENTER;
	  v = CENTER;
	  o = 0;

	  if (ds == 0x10) {
	    bmp_drawstring(bm, pts[0], pts[1], h, v, o, a, str, RGB(0xff,0x80,0x40));
	  }
	  toff += slen + 10;
	} else if (st == 6) {
	  int sp;

	  /* 7.4.2.1.7 Symbol+String 
	   *   00 [B:B:B] Additional Background Info
	   *     11    Auxiliary Data Flag
	   *     13    Additional Background Info Flag
	   *     14:15 Additional Background Info Type
	   *   02 [N:NX]  X Coordinate
	   *     00:12 X-coordinate
	   *     13:15 Relative position within integrated parcel
	   *   04 [N:NX]  Y Coordinate
	   *     00:12 Y-coordinate
	   *     13:15 Relative position within integrated parcel
	   *   06 [B:B] String Placement
	   *   08 Character Information Data List
	   *   xx Altitude Information
	   */
	  sp = _2b(ptr + toff + 6);
	  slen = _2b(ptr + toff + 8)*2;
	  memcpy(str, ptr + toff + 10, slen);
	  zprintf("  string6: '%s' x:%d y:%d  align:%d placement:%d\n", str, xc, yc, 
		  extract(sp, 14, 15),
		  extract(sp, 12, 13));
	  bmp_drawstring(bm, pts[0], pts[1], CENTER, CENTER, 0, 0, str, RGB(0xff,0x80,0x40));
	  toff += slen + 10;
	} else {
	  os_dump(ptr + toff, 32);
	  exit(0);
	}
      }
    }
  }
  zprintf("=========== @nameend\n");
}

// 7.3.2.2.1 Minimum Graphics Record
struct mingr_t
{
  uint16_t hdr;   // [B:B:B:SWS]   size
  uint16_t flag;  // [B:B:B:B:B:N] ncoord
  uint16_t code;  // [N]           type code
  uint16_t addl;  // [B:B:B:B:N]   name field,auxdata
  uint16_t sx;    // [N:NZ]        starting x
  uint16_t sy;    // [N:NZ]        starting y
  struct {
    int8_t xo;    // [I]           deltax
    int8_t yo;    // [I]           deltay
  } _PACKED coords[1];
  // uint16_t nameoff;
  // uint16_t addoff;
} _PACKED;

void osmtag(int type)
{
  char *kv = "  <tag k=\"%s\" v=\"%s\" ";

  switch (type) { 
  case 0x121:
  case 0x122:
  case 0x123:
  case 0x124:
    fprintf(stderr, kv, "natural", "water");
    break;
  case 0x132:
    fprintf(stderr, kv, "administrative", "boundary");
    fprintf(stderr, " admin_level=\"4\"");
    break;
  case 0x210:
    fprintf(stderr, kv, "highway", "motorway");
    break;
  }
  fprintf(stderr, ">\n");
}

#define ZZ -1
int drawme;

void dumpbkgd(struct lmr_t *lmr, void *ptr, size_t len)
{
  size_t hlen = SWS(_2b(ptr)); // 7.3.1 [SWS] background distribution header size
  off_t off, poff, boff;
  int i, n, p[256], t[256], j, plen, val, ncoord, xc, yc, k, mconst;
  struct mingr_t *gr;
  int *pts;
  static int ns;

  zprintf("========= Background %.4x\n", len);
  for (off=2; off<hlen; off+=4) {
    poff = D(_2b(ptr + off));         // 7.3.1.1 [D] element unit offset
    plen = SWS(_2b(ptr + off + 2));   // 7.3.1.1 [SWS] element unit size
    if (poff != 0xFFFF) {
      zprintf("  Element: %.4x %.4x [%.4x]\n", poff, plen, poff+plen);

      n = _2b(ptr + poff);           // 7.3.2 [N] number of background types
      poff += 2;
      for  (i=0; i<n; i++) {
	/* 7.3.2.1 Background Type Unit 
	 *   00 [D] Background Unit type offset
	 *   02 [B:B:N] Shape Classification+#MinGrRec
	 *     00:11 Number of Minimum Graphics Data Records
	 *     13    Height Flag
	 *     14:15 Shape Classification
	 */
	boff = D(_2b(ptr + poff));    // 7.3.2.1 [D] Background unit type offset
	val =  _2b(ptr + poff + 2);   // 7.3.2.1 [B:B:N] Shape Class+#MinGrRec
	p[i] = extract(val, 0, 11);   // 7.3.2.1 [N] # min graphics records
	t[i] = extract(val, 14, 15);  // 7.3.2.1 [B] Shape Class [0=point,1=line,2=poly]
	zprintf("     Shape:%x height:%x #gr:%d offset:%.4x\n",
	       extract(val, 14, 15),
	       extract(val, 13, 13),
	       p[i],
	       poff+boff);
	poff += 4;
      }

      for (i=0; i<n; i++)  {
	/*  7.3.2.2 Minimum Graphics Data List */
	for (j=0; j<p[i]; j++) {
	  /* 7.3.2.2.1 Minumum Graphics Data Record
	   *   00 [B:B:B:SWS] Header
	   *     00:11 Size of Minimum Graphics Data Record
	   *     13    ExtData Flag
	   *     14    Temporal Data Flag
	   *   02 [B:B:B:B:B:N] Display Scale Flag+Number of Coords
	   *     00:10 Number of Offset Coordinate Records
	   *     11:15 Display Scale Flag X
	   *   04 [N] Type Code
	   *   06 [B:B:B:B:N] Additional Background Information
	   *     00:02 Multiplication constant
	   *     09    Underground Attribute Flag
	   *     10    Pen-up Flag
	   *     11    AuxData Flag
	   *     12    Name Flag
	   *     13    Additional Background Info Flag
	   *     14:15 Additional Background Info Type
	   */
	  gr = ptr + poff;
	  swapw(&gr->hdr);
	  swapw(&gr->flag);
	  swapw(&gr->code);
	  swapw(&gr->addl);
	  swapw(&gr->sx);
	  swapw(&gr->sy);

	  plen = SWS(extract(gr->hdr, 0, 11));
	  ncoord = extract(gr->flag, 0, 10); // 7.3.2.2.1 [N] Number of Offset Coordinates (0..2047)
	  mconst = 1L << extract(gr->addl, 0, 2);
	  zprintf("      lvl:%2d gr%.3d,%.3d shape:%d #coord:%.4d type:%.4x addl:%.4x rel:%2x,%2x [%s]\n", 
		 extract(lmr->header, 10, 15),
		 i, j, t[i],
		 ncoord, gr->code, gr->addl,
		 extract(gr->sx, 13, 15),
		 extract(gr->sy, 13, 15),
		 lookup(types, gr->code));
	  rdtype[gr->code]++;

	  /* Check extended length of struct */
	  xc = poff + 12 + ncoord*2;
	  if (extract(gr->addl, 12, 12))
	    xc+=2; // name
	  if (extract(gr->addl, 11, 11))
	    xc+=2; // addl
	  if (xc != poff + plen) {
	    fprintf(stderr, "mismatch gr length: %.5x %.5x\n", xc, poff + plen);
	    exit(0);
	  }
	  if (t[i]) {
	    int rx,ry;

	    /* Extract coordinates */
	    rx = extract(gr->sx, 13, 15);           // 0..7
	    ry = extract(gr->sy, 13, 15);           // 0..7
	    xc = extract(gr->sx, 0, 12) + rx*4096;  // 0..8191
	    yc = extract(gr->sy, 0, 12) + ry*4096;  // 0..8191

	    pts = zmalloc(sizeof(int)*2*(ncoord+2));
	    setxy(pts,xc,yc);

	    for (k=0; k<ncoord; k++) {
	      xc += gr->coords[k].xo * mconst;
	      yc += gr->coords[k].yo * mconst;
	      setxy(pts+k*2+2,xc,yc);
	    }
	    if (t[i] == 2) {
	      bmp_polyfill(bm, ncoord, pts, typecolor(gr->code));
	    }
	    else {
	      bmp_polyline(bm, ncoord+1, pts, typecolor(gr->code));
	    }
	    bmp_setstyle(bm, NULL);
	    free(pts);
	  }
	  poff += plen;
	}
      }
    }
  }
  zprintf("== end background\n");
}

int typecolor(int type)
{
  switch (type) {
  case 0x121:
  case 0x122:
  case 0x123:
  case 0x124: // water
    return RGB(0,0,0xFF);
  case 0x128: // island
    return RGB(0,0xFF,0);
  case 0x131: // address, country
    return RGB(0x40,0x40,0x40);
  case 0x132: // address, state
    return RGB(0x80,0x80,0x80);
  case 0x134: // address, municipality
    return RGB(0x50,0x50,0x50);
  case 0x140: // urban district
    return RGB(0xff,0x58,0x00);
  case 0x141: // green belt, park
    return  RGB(0,0xFF,0);
  case 0x142: // urban district
    return  RGB(0,0xFF,0xFF);
  case 0x210:
  case 0x211: // road type x
    return RGB(0xFF,0,0);
  case 0x242: // very high speed rail
    bmp_setstyle(bm,"stroke-dasharray: 9,5;stroke-width:2;");
    return RGB(0xff,0xff,0xff);
  case 0x408: // cemetery
    return RGB(0x40,0x40,0x40);
  case 0x464: // university,college
    return RGB(0xff,0x00,0xff);
  case 0x620: // shopping center
    return RGB(0xff,0x99,0x00);
  case 0x6180: // golf course
    return RGB(0x0,0x80,0x0);
  default:
    printf("@@type: %x\n", type);
    return RGB(0xff,0xff,0xff);
  }
  return -1;
}

void showmap(struct lmr_t *lmr, void *map, size_t len)
{
  struct mfde_t *de;
  struct mapframe_t *mf = map;
  int nData, nExt, j, i;
  int nx, ny, lvl, cx, cy;

  /* Basic map/Extended map numbers */
  lvl = extract(lmr->header, 10, 15);
  nData = extract(lmr->numbers, 12, 15);
  nExt  = extract(lmr->numbers, 8, 11);

  swapw(&mf->size);
  swapw(&mf->llcode);
  swapw(&mf->dipid);
  swapl(&mf->pmcode);
  swapw(&mf->dsflag);
  swapw(&mf->rlx);
  swapw(&mf->rly);
  swapw(&mf->geo_str);
  swapw(&mf->geo_dec);
  swapl(&mf->rg_addr);
  swapw(&mf->rg_size);
  swapw(&mf->nregion);

  de = (void *)&mf[1] + mf->nregion * 4;

  cx = extract(mf->llcode, 0, 7);
  cy = extract(mf->llcode, 8, 15);
  nx = (1+lmr->nblocksets.lng)*(1+lmr->nblocks.lng)*(1+lmr->nparcels[0].lng);
  ny = (1+lmr->nblocksets.lat)*(1+lmr->nblocks.lat)*(1+lmr->nparcels[0].lat);

  /* [N] Map Distribution Header
   * [VAR] Main Map Basic Data Frame
   * [VAR] Main Map Extended Data Frame
   */
  _mx = geo_secs(mf->llpid.lng);
  _my = geo_secs(mf->llpid.lat);

#if 0
  zprintf("    ==== [%d,%d] lat/lng : @@ lvl:%2d ", cx, cy, lvl);
  showpid(mf->llpid);
  printf(" to ");
  printf("%lf,%lf\n", _my + (_ry-_ly)/ny, _mx + (_rx-_lx)/nx);

  printf("    divint:%d adj:%d type:%d lat:%d lng:%d\n",
	 extract(mf->dipid, 14, 15),
	 extract(mf->dipid, 13, 13),
	 extract(mf->dipid, 8, 9),
	 extract(mf->dipid, 4, 7),
	 extract(mf->dipid, 0, 3));
#endif
  for (i=0; i<nData; i++) {
    swapl(&de[i].offset);
    swapw(&de[i].size);
  }
  /* Draw Background, Road, Names */
  if (de[1].size) {
    dumpbkgd(lmr, map + D(de[1].offset), SWS(de[1].size));
  }
  if (de[0].size) {
    dumproad(map + D(de[0].offset), SWS(de[0].size));
  }
  if (de[2].size) {
    dumpname(map + D(de[2].offset), SWS(de[2].size));
  }
}

/* A = blockset, NA = nblocksets.lat
 * B = block,    NB = nblocks.lat
 * C = parcel,   NC = nparcels[0].lat
 * P = subparcel,NP = nparcels[x].lat
 */
void divbsmr(int *x, int *y, int a, int na, int b, int nb, int c, int nc, int p, int np)
{
  *x = a%na;
  *y = a/na;
  if (b != -1) {
    *x = (*x)*nb + (b%nb);
    *y = (*y)*nb + (b/nb);
  }
  if (c != -1) {
    *x = (*x)*nc + (c%nc);
    *y = (*y)*nc + (c/nc);
  }
  if (p != -1) {
    *x = (*x)*np + (p%np);
    *y = (*y)*np + (p/np);
  }
}

int isin(double y, double x, double ly, double lx, double dy, double dx)
{
  return (y >= ly && y <= (ly+dy) && x >= lx && x <= (lx+dx));
}

/* Dump Block Management Table entry */
int showbmt(int fd, struct lmr_t *lmr, struct bsmr_t *bsmr, int block, void *pdat, int poff, int parent)
{
  off_t mapoff;
  uint32_t add, size;
  struct parman_t *pi;
  mapinfo_t  *mi;
  int pt, lt, k, j, lvl, bset, nx, ny, mx, my;
  void *mdat;
  int *ip;

  lvl = extract(lmr->header, 10, 15);
  bset = extract(bsmr->header, 0, 7);

  nx = (1+lmr->nblocksets.lng)*(1+lmr->nblocks.lng)*(1+lmr->nparcels[0].lng);
  ny = (1+lmr->nblocksets.lat)*(1+lmr->nblocks.lat)*(1+lmr->nparcels[0].lat);
  _my = (_ry - _ly) / ny;
  _mx = (_rx - _lx) / nx;

  pi = (void *)(pdat + poff);
  mi = (void *)&pi[1];
  swapw(&pi->type);
  swapw(&pi->routeoff);

  /* Get Parcel Type, List Type */
  pt = extract(pi->type, 8, 9);
  lt = extract(pi->type, 0, 7);
  assert(lt == 0);
  k = (1+lmr->nparcels[pt].lat)*(1+lmr->nparcels[pt].lng);
  zprintf("  list type:%2d  parcels[%d]:%d\n", lt, pt, k);
  
  /* Loop through nparcels x nparcels, calculate integrated packets */
  ip = zmalloc(sizeof(int)*k);
  for (j=0; j<k; j++) {
    int l;

    add  = _4b(&mi->map0[j].dsa);
    size = _2b(&mi->map0[j].size);
    if (ip[j] || add == 0xFFFFFFFF)
      continue;
    ip[j] = -1;
    if (size) {
      ip[j] = j+1;
      for (l=j+1; l<k; l++) {
	if (add == _4b(&mi->map0[l].dsa)) {
	  ip[l] = j+1;
	}
      }
    }
    zprintf(" parcel:%4d ip:%4d\n",  j, ip[j]);
  }

  /* Loop through all packets */
  for (j=0; j<k; j++) {
    add = _4b(&mi->map0[j].dsa);
    size = _2b(&mi->map0[j].size);
    if (add == 0xffffffff)
      continue;

    mx = my = 0;
    divbsmr(&mx, &my, 
	    bset, 1+lmr->nblocksets.lat,
	    block, 1+lmr->nblocks.lat,
	    parent >=0 ? parent : j,  1+lmr->nparcels[0].lat,
	    parent >=0 ? j      : -1, 1+lmr->nparcels[0].lat);
    zprintf("    lvl:%2d.%d blockset:%2d block:%3d parcel:%4d %.8lx %.4x ip:%4d [%4d,%4d]{%4d,%4d} %lf,%lf to %lf,%lf]",
	   lvl, pt, bset, block, 
	   j, add, size, ip[j],
	   mx, my,
	   nx, ny,
	   _ly+_my*my, _lx+_mx*mx, _ly+(my+2)*_my, _lx+(mx+2)*_mx);

    if (parent != -1) {
      zprintf(" parent: %4d", parent);
    }
    zprintf("\n");
    drawme = 0;
    if (size) {
      if (isin(30.310801, -97.711401, _ly+_my*my, _lx+_mx*mx, _my, _mx)) {
	printf("l:%2d pt:%2d  bs:%2d block:%2d parcel:%4d @@@ AUSTIN\n", lvl, pt, bset, block, j);
	printf("[%lf,%lf] to [%lf,%lf]\n",
	       _ly+_my*my, _lx+_mx*mx, _ly+(my+1)*_my, _lx+(mx+1)*_mx);
	drawme = 1;
      }
    }

    if (!size) {
      /* Parse subparcel */
      showbmt(fd, lmr, bsmr, block, pdat, D(add), j);
    } else if (drawme) {
      bmsz = 4096;
      m = S(bmsz / 4096.0, bmsz / 4096.0);
      bm = bmp_allocsvg(bmsz+1,bmsz+1,"out.svg");
      bmp_rect(bm,0,0,bmsz,bmsz,RGB(0xFF,0xFF,0));

      mapoff = getsector(add);
      mdat = zreado(fd, size * logical_sz, mapoff, "map");
      showmap(lmr, mdat, size * logical_sz);
      zfree(mdat, size * logical_sz);
      
      bmp_write(bm, "poly.bmp");
      exit(0);
    }
  }
  zprintf("  end list: %d\n", pt);
  free(ip);
}

void showalldata()
{
  int fd, i, j, lvl, bset, block, nx, ny;
  struct datavol_t dv;
  struct mhr_t *mhr;
  off_t moff;
  void *zdat[34];
  char name[64];
  struct pdmdh_t *pdmdh;
  struct lmr_t *lmr, **lmrmap;
  struct bsmr_t *bsmr;
  matrix_t s;

  fd = open("audi/ALLDATA.KWI", O_LARGEFILE|O_RDONLY);
  if (fd < 0)
    return;
  fend = lseek(fd, 0, SEEK_END);
  lseek(fd, 0, SEEK_SET);

  read(fd, &dv, sizeof(dv));
  swapw(&dv.spec_mid.date);
  swapw(&dv.data_mid.date);
  swapw(&dv.system_mid.date);
  swapw(&dv.log_size);
  swapw(&dv.sector_size);
  swapw(&dv.background);
  swapw(&dv.contents[0]);
  swapw(&dv.contents[1]);
  swapw(&dv.contents[2]);
  swapw(&dv.contents[3]);

  sector_sz = dv.sector_size;
  logical_sz = dv.log_size;

  printf("system specific: %s\n", dv.spec_ssi);
  showmid(dv.spec_mid);
  printf("data author: %s\n", dv.data_ssi);
  showmid(dv.data_mid);
  printf("system: %s\n", dv.system_ssi);
  showmid(dv.system_mid);
  printf("fmt:%s\ndata:%s\ntitle:%s\nmedia_ver:%s\n", dv.format_ver, dv.data_ver, dv.disk_title, dv.media_version);
  printf("main map   : %d\n", dv.contents[0] & (1L << 15) ? 1 : 0);
  printf("route plan : %d\n", dv.contents[0] & (1L << 14) ? 1 : 0);
  printf("index data : %d\n", dv.contents[0] & (1L << 13) ? 1 : 0);

  printf("lower left: ");
  showpid(dv.box_ll);
  printf("\nupper right: ");
  showpid(dv.box_ur);
  printf("\n");
  printf("logical sector: %d\n", dv.log_size);
  printf("sector: %d\n", dv.sector_size);
  printf("background: %x\n", dv.background);
  printf("Background in map: %s\n", dv.background & (1L << 15) ? "sea" : "land");
  printf("Background out of map: %s\n", dv.background & (1L << 14) ? "sea" : "land");

  memset(name, 0, sizeof(name));
  printf("levels---\n");
  os_dump(dv.level_mgmt, sizeof(dv.level_mgmt));

  /* Record 1: PRDM
   * Record 2: RRDM
   * Record 3: Index Data Management
   * Record 4: Various Parameters
   * Record 6: Graphics Data management
   * Record 7: Voice Data Management
   * Record 19:
   * Record 21:
   * Record 22:
   * Record 27:
   * Record 30:
   */
  mhr = zread(fd, 34 * sizeof(struct mhr_t));
  for (i=0; i<34; i++) {
    strcpy(name, "/media/");
    swapl(&mhr[i].dsa);
    swapw(&mhr[i].size);
    strncat(name, mhr[i].name, 12);
    printf("-- Record %.2d ", i+1);
    printf("  Addr:%.8" PRIx32 " Size:%.4x  Name:'%s'\n", mhr[i].dsa, mhr[i].size, name);
    if (mhr[i].name[0] == 0 && mhr[i].dsa != -1) {
      moff = getsector(mhr[i].dsa);
      zdat[i] = zreado(fd, mhr[i].size * logical_sz, moff, "mhr");
      //os_dump(zdat[i], mhr[i].size * logical_sz);
    }
  }

  /* 6.1 Dump PDMR
   *    Header
   *    Level Management Records
   *    Block Set Management Records
   *    Block Management Tables 
   */
  moff = sizeof(struct pdmdh_t);
  pdmdh = zdat[0];
  swapw(&pdmdh->size);
  swapw(&pdmdh->lmr_sz);
  swapw(&pdmdh->bsmr_sz);
  swapw(&pdmdh->bmr_sz);
  swapw(&pdmdh->nlmr);
  swapw(&pdmdh->nbsmr);
  swapw(&pdmdh->filename);

  /* This gets divided by # of parcels/packets */
  printf("lower left:\n");
  printlatlng(pdmdh->lower_lat, pdmdh->left_lng);

  printf("\nupper right:\n");
  printlatlng(pdmdh->upper_lat, pdmdh->right_lng);

  _lx = geo_secs(pdmdh->left_lng);
  _rx = geo_secs(pdmdh->right_lng);
  _ly = geo_secs(pdmdh->lower_lat);
  _ry = geo_secs(pdmdh->upper_lat);

  printf("\nlmr_sz: %x  bsmr_sz:%x  bmr_sz:%x nlmr:%x nbsmr:%x filename:%x\n",
         SWS(pdmdh->lmr_sz), pdmdh->bsmr_sz, pdmdh->bmr_sz, pdmdh->nlmr, pdmdh->nbsmr, pdmdh->filename);

  /* 6.1.1 Dump level records */
  lmrmap = calloc(pdmdh->nlmr, sizeof(struct lmr_t *));
  for (i=0; i<pdmdh->nlmr; i++) {
    lmr = (struct lmr_t *)(zdat[0] + moff);

    lmrmap[i] = lmr;
    swapw(&lmr->header);
    swapw(&lmr->numbers);
    swapl(&lmr->dispflag[0]);
    swapl(&lmr->dispflag[1]);
    swapl(&lmr->dispflag[2]);
    swapl(&lmr->dispflag[3]);
    swapl(&lmr->dispflag[4]);
    swapw(&lmr->bsmr_off);
    swapw(&lmr->nrsize);

    lvl = extract(lmr->header, 10, 15);
    nx = (1+lmr->nblocksets.lng)*(1+lmr->nblocks.lng)*(1+lmr->nparcels[0].lng);
    ny = (1+lmr->nblocksets.lat)*(1+lmr->nblocks.lat)*(1+lmr->nparcels[0].lat);

    printf("======== lmr%d:  level=%d  upper=%d lower=%d\n", i, 
           lvl,
           extract(lmr->header, 4, 7),
           extract(lmr->header, 0, 3));
    printf("  map size        : %dx%d [%10lf,%10lf]\n",nx,ny,(_rx-_lx)/nx, (_ry-_ly)/ny);
    for (j=0; j<5; j++) {
      printf(" disp%d: %d\n", 1+j, lmr->dispflag[j]);
    }
    printf(" latlng block sets: %dx%d\n", 
           1+lmr->nblocksets.lng, 1+lmr->nblocksets.lat);
    printf(" latlng blocks    : %dx%d\n",
           1+lmr->nblocks.lng, 1+lmr->nblocks.lat);
    for (j=0; j<=3; j++) {
      printf(" latlng parcels%d  : %dx%d\n", j, 1+lmr->nparcels[j].lat, 1+lmr->nparcels[j].lng);
    }
    printf(" bsmroff:         : %d\n",
           lmr->bsmr_off * 2);
    printf(" node record size : %d\n", lmr->nrsize * 2);

    /* Display map decoding info */
    printf("  #basic map      : %d\n", extract(lmr->numbers, 12, 15));
    printf("  #extend map     : %d\n", extract(lmr->numbers, 8, 11));
    printf("  #basic route    : %d\n", extract(lmr->numbers, 4, 7));
    printf("  #extend route   : %d\n", extract(lmr->numbers, 0, 3));
    if (SWS(pdmdh->lmr_sz) >= sizeof(lmr_t)+2) {
      void *lext;
      int   loff, xt;

      /* Extended info */
      lext = &lmr[1];
      xt = _2b(lext);
      printf("  #road           : %d\n", extract(xt, 10, 13)+1); // 7.2 Road Data Frame       1..16
      printf("  #background     : %d\n", extract(xt, 5, 9)+1);   // 7.3 Background Data Frame 1..32
      printf("  #name           : %d\n", extract(xt, 0, 4)+1);   // 7.4 Name Data Frame       1..32
#ifdef DEBUG
      loff = 2;
      for  (j=0; j<extract(xt, 10, 13)+1; j++) {
	printf("   road%2d: %.4x\n", j, _2b(lext + loff));
	loff += 2;2
      }
      for  (j=0; j<extract(xt, 5, 9)+1; j++) {
	printf("   bkgd%2d: %.4x\n", j, _2b(lext + loff));
	loff += 2;
      }
      for  (j=0; j<extract(xt, 0, 4)+1; j++) {
	printf("   name%2d: %.4x\n", j, _2b(lext + loff));
	loff += 2;
      }
#endif
    }
    moff += SWS(pdmdh->lmr_sz);

  }

  /* 6.1.2 Dump Block Set Management Records */
  bsmr = (struct bsmr_t *)(zdat[0] + moff);
  for (i=0; i<pdmdh->nbsmr; i++) {
    swapw(&bsmr->header);
    swapl(&bsmr->bmt_offset);
    swapl(&bsmr->bmt_size);

    lvl = extract(bsmr->header, 10, 15);
    bset = extract(bsmr->header, 0, 7);
    lmr = findlevel(lmrmap, pdmdh->nlmr, lvl);
    
    printf("---- bsmr%d: level=%2d blockset=%3d [%dx%d [%dx%d] [%dx%d] offset:%.8lx size:%.4x\n",
	   i, lvl, bset, 1+lmr->nblocksets.lng, 1+lmr->nblocksets.lat,  
	   1+lmr->nblocks.lng, 1+lmr->nblocks.lat, 
	   1+lmr->nparcels[0].lng, 1+lmr->nparcels[0].lat,
	   bsmr->bmt_offset, SWS(bsmr->bmt_size));
    if (bsmr->bmt_size) {
      struct bmt_t *bmt;
      int   nblocks;

      /* Loop through all blocks in this blockset (nblocks.lat * nblocks.lng) 
       *    00 [D]  addres of parcel management info
       *    04 [BS] size of parcel management info (* logical sector size)
       */
      nblocks = (1+lmr->nblocks.lat)*(1+lmr->nblocks.lng);
      assert(nblocks*6 == SWS(bsmr->bmt_size));

      /* Use bmt or bmtfile here */
      bmt = (struct bmt_t *)(zdat[0] + D(bsmr->bmt_offset));
      for(block=0; block<nblocks; block++) {
	int nparcels;

        swapl(&bmt->dsa);
        swapw(&bmt->size);

	nparcels = (1+lmr->nparcels[0].lat)*(1+lmr->nparcels[0].lng);

	zprintf(" block:%2d dsa:%.08lx size:%.4x nparcels:%d\n", block, bmt->dsa, bmt->size * logical_sz, nparcels);
	if (bmt->size && lvl==0) {
	  off_t poff;
	  void *pdat;

	  poff = getsector(bmt->dsa);
	  pdat = zreado(fd, bmt->size * logical_sz, poff, "pdat");
	  showbmt(fd, lmr, bsmr, block, pdat, 0, -1);
	  zfree(pdat, bmt->size * logical_sz);
	}
	bmt++;
      }
    }
    bsmr++;
  }
  //bmp_write(bm, "poly.bmp");
  showrt(rdtype);

  printf("total sections: %d\n", nfext);
  exit(0);
}

int main(int argc, char *argv[])
{
  int fd, i, j;
  void *ptr;
  off_t end, offset;
  struct lmm_t  lmm;
  struct sii_t *sii;
  struct mii_t *mii;
  struct mmi_t *mmi;

  setbuf(stdout, NULL);
  showalldata();

  if ((fd = open("/media/LOADING.KWI", O_RDONLY)) < 0) {
    perror("open");
    return -1;
  }
  read(fd, &lmm, sizeof(lmm));
  lmm.n = ntohs(lmm.n);

  /* Read all SII */
  sii = zread(fd, lmm.n * sizeof(struct sii_t));
  for (i=0; i<lmm.n; i++) {
    swapw(&sii[i].nm);
    swapw(&sii[i].mid.date);
    printf("sii%d: nm=%d\n", i+1, sii[i].nm);
    showmid(sii[i].mid);

    /* Read NM * mii */
    mii = zread(fd, sii[0].nm * sizeof(struct mii_t));
    for (j=0; j<sii[i].nm; j++) {
      printf(" mii%d.%d cat:%x name:%s version:%s\n", 
             i,j,mii[j].category, mii[j].modulename, mii[j].modulever);
      printf("   category:%s %s %s\n",
             mii[j].category & (1L << 7) ? "diag" : "",
             mii[j].category & (1L << 6) ? "test" : "",
             mii_category[mii[j].category & 3]);
    }

    /* Read NM * mmi */
    mmi = zread(fd, sii[0].nm * sizeof(struct mmi_t));
    for (j=0; j<sii[i].nm; j++) {
      mmi[j].date_start = ntohs(mmi[j].date_start);
      mmi[j].date_end = ntohs(mmi[j].date_end);
      mmi[j].addr = ntohl(mmi[j].addr);
      mmi[j].size = ntohs(mmi[j].size);
      printf(" mmi%d.%d start:%x end:%x title:%s start:%x size:%d\n", 
             i,j,mmi[j].date_start, mmi[j].date_end, mmi[j].title, mmi[j].addr, 
             mmi[j].size * 2048);
      if (mmi[j].date_start) {
        printf("   start date: ");
        showdate(mmi[j].date_start);
      }
      if (mmi[j].date_end) {
        printf("   end date: ");
        showdate(mmi[j].date_end);
      }
      os_dump(mmi[j].mandep, sizeof(mmi[j].mandep));
      dump_mandep(fd, mmi[j].mandep);
    }
  }
  //dumpbm(fd);
}
