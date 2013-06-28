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
//#include <oslib.h>
#include "bmp.h"

#define _PACKED __attribute__((packed))
#pragma pack(1)

int isin(double y, double x, double ly, double lx, double dy, double dx);

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
  return "";
}

struct key_t types[] = {
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
  { 0x210, "road type 0" },
  { 0x211, "road type 1" },
  { 0x242, "very high speed railway, JR line [main line]" },
  { 0x280, "other airport" },
  { 0x408, "cemetery" },
  { 0x43c, "national defense facility, base" },
  { 0x464, "university, college" },
  { 0x480, "hospital" },
  { 0x520, "other sports facility" },
  { 0x620, "other shopping facility" },
  { 0x6180, "golf course" },
  { -1 },
};

/* Decode 1,2,3,4 BigEndian bytes */
uint8_t _1b(void *v)
{
  return *(uint8_t *)v;
}

uint16_t _2b(void *v)
{
  uint8_t *s = v;

  return ((s[0] << 8) + s[1]);
}

uint32_t _3b(void *v)
{
  uint8_t  *s = v;

  return ((s[0] << 16) + (s[1] << 8) + s[2]);
}

uint32_t _4b(void *v)
{
  uint8_t *s = v;

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
struct sectoraddress
{
  uint32_t addr;
}  _PACKED;

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

off_t getsector(struct sectoraddress sa)
{
#if 0
  printf("GetSector: %c%c%d.%d\n",
	 sa.addr & 0x80 ? 'B' : 'A',
	 sa.addr & 0x40 ? 'D' : 'S',
	 sa.addr  >> 8,
	 sa.addr & 0x3F);
#endif
  return (sa.addr >> 8) * sector_sz + (sa.addr & 0x3F) * logical_sz;
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
  printgeo(pid.lat, LATITUDE);
  printf(" ");
  printgeo(pid.lng, LONGITUDE);
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

void *zread(int fd, int sz)
{
  void *ptr;

  ptr = zmalloc(sz);
  read(fd, ptr, sz);
  return ptr;
}

size_t nm;

void zfree(void *p, size_t sz)
{
  nm -= sz;
  free(p);
}

void *zreado(int fd, size_t sz, off_t off, char *lbl)
{
  void *ptr;

  nm += sz;
  ptr = zmalloc(sz);
  pread(fd, ptr, sz, off);
  return ptr;	
}

const char *mii_category[] = { "initial program", "program", "library", "data" };

void swapw(uint16_t *w)
{
  *w = ntohs(*w);
}

void swapl(uint32_t *l)
{
  *l = ntohl(*l);
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
      fdo = open(name, O_RDWR|O_CREAT);
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
  struct sectoraddress 	dsa;
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

  uint16_t		nlmr;          // [N]
  uint16_t		nbsmr;         // [N]

  /* lmr_t        lmr[var];    Level Management Records
   * bsmr_t       bsmr[var];   Block Set Management Records
   * bmt_t        bmt[var];    Block Management Tables
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

/* Reference Record of Parcel Management Information */
struct bmt_t
{
  struct sectoraddress  dsa;           // [DSA] sector address or reference
  uint16_t              size;          // [BS]  size in logical sectors
  uint8_t               filename[12];  // [C]   optional
} _PACKED;

/* Parcel Management Info */
struct parman_t
{
  uint16_t             type;       // [N:N] Parcel Management type
  uint16_t             routeoff;   // [D] route guidance parcel management list
  // mapinfo_t mapdata[];
};

/* Main Map Parcel Management Record */
union mapinfo_t {
  struct {
    struct sectoraddress dsa;      // [DSA] sector address
    uint16_t             size;     // [BS] size in logical sectors
  } _PACKED map0;
  struct {
    struct sectoraddress dsa;
    uint16_t             size1;
    uint16_t             size2;
  } _PACKED map1;
  struct {
    struct sectoraddress dsa;
    uint16_t             size1;
    uint16_t             size2;
  } _PACKED map2;
  struct {
    struct sectoraddress dsa;
    uint16_t             size;
    char                 filename[12];
  } _PACKED map100;
} _PACKED;

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

uint32_t extract(uint32_t val, int start, int end)
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
  struct sectoraddress rg_addr; // [DSA]
  uint16_t             rg_size; // [BS]
  uint16_t             nregion; // [N]
  /* [VAR] Regions x4 */
  /* [VAR] mfde_t[] */
} _PACKED;

bitmap_t *bm;
matrix_t  m;

/* Multilink Header */
struct mlh_t
{
  uint32_t mmh;
  uint16_t nmh;
  uint16_t shape;
  uint16_t addl;
  uint16_t alti;
  uint16_t passage;
} _PACKED;

double   ssz  = 8192.0;
int      bmsz = 4000;

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
void dumproad(void *ptr, size_t len)
{
  size_t hlen = SWS(_2b(ptr));
  int ninter, ndc, nad, lvl, i, off, xoff, xsz, npoly, poff, last, j, hdr, noff, nlen, lattr;
  int sx, sy, xc, yc, rx, ry, k, nnodes, nip, l, no;
  int pts[20000],npts;
  vector_t v;

  /* 7.2.1 */
  ninter = _2b(ptr + 2);
  ndc = _1b(ptr + 4);
  nad = _1b(ptr + 5);
  lvl = extract(_2b(ptr + 6), 10, 15);
  printf("========== road %.4x inter:%.4x  display:%2d additional:%d lvl:%d\n", len, ninter, ndc, nad, lvl);

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
    npoly = extract(_2b(ptr + off + 2), 0, 11); // Number of Polylines (0..4095)

    if (xoff != 0xFFFF) {
      printf("  display:%.4x  npoly:%.4x display flag:%.4x\n", xoff, npoly, _2b(ptr + xoff));
      xoff += 2;
      for (j=0; j<npoly; j++) {
	/* 7.2.2.1.1.1 MultiLink Header 
	 *    00 [B:B:B:B:SWS:B:SWS] Header
	 *    04 [N] Node Information Management Header
	 *    06 [SWS] Shape Information Header [x]
	 *    08 [SWS] Additional Node Information Header [y]
	 *    10 [SWS] Altitude Information Management Header [z]
	 *    12 [SWS] Passage Information Management Header [w]
	 *    
	 *    [x] Shape Information
	 *    [x] Link/Node Connection Information
	 *    [y] Additional Node Information
	 *    [z] Altitude Information
	 *    [w] Passage Regulation Information
	 */
	hdr = _4b(ptr + xoff);
	nnodes = extract(_2b(ptr + xoff + 4), 0, 10);
	printf("    poly%d: %.4x  nodes:%x shape:%.4x additional:%.4x altitude:%.4x passage regulation:%.4x hdr.size:%x size:%x\n", 
	       j, xoff,
	       nnodes,
	       SWS(extract(_2b(ptr + xoff + 6),0,11)),  // shape
	       SWS(extract(_2b(ptr + xoff + 8),0,11)),  // additional
	       SWS(extract(_2b(ptr + xoff + 10),0,11)), // altitude
	       SWS(extract(_2b(ptr + xoff + 12),0,11)), // passage regulation
	       SWS(extract(hdr, 0,7)),SWS(extract(hdr,16,27)));
	lattr = _2b(ptr + xoff + 14);
	printf("  road type:%d link:%d infra:%d rte:%d  toll:%d\n",
	       extract(lattr, 12, 15),
	       extract(lattr, 11, 11),
	       extract(lattr, 10, 10),
	       extract(lattr, 9, 9),
	       extract(lattr, 8, 8));

	/* Get offset of Shape Data */
	noff = SWS(extract(hdr,0,7));
	nlen = SWS(extract(_2b(ptr + xoff + 6), 0, 11));

	npts = 0;
	no = noff;
	for (k=0; k<nnodes; k++) {
	  /* link attribute */
	  lattr = _2b(ptr + xoff + noff);
	  nip = extract(lattr, 0, 9);
	  printf("  oneway:%d.%d planned:%d tunnel:%d bridge:%d ip:%d\n",
		 extract(lattr, 15, 15),
		 extract(lattr, 13, 14),
		 extract(lattr, 12, 12),
		 extract(lattr, 11, 11),
		 extract(lattr, 10, 10),
		 nip);
	  sx = _2b(ptr + xoff + noff + 2);
	  sy = _2b(ptr + xoff + noff + 4);
	  rx = extract(sx, 13, 15);
	  ry = extract(sy, 13, 15);
	  xc = extract(sx, 0, 12) + rx*4096;
	  yc = extract(sy, 0, 12) + ry*4096;

	  //printf("    xc:%4d yc:%4d\n", xc, yc);
	  noff += 6;

	  v = matxvec(m,vecinit(xc,yc));
	  pts[npts++] = v.v[0];
	  pts[npts++] = v.v[1];
	  for (l=0; l<nip; l++) {
	    xc += (char)_1b(ptr + xoff + noff);
	    yc += (char)_1b(ptr + xoff + noff+1);

	    v = matxvec(m,vecinit(xc,yc));
	    pts[npts++] = v.v[0];
	    pts[npts++] = v.v[1];
	    //printf("    xc:%4d yc:%4d\n", xc, yc);
	    noff += 2;
	  }
	}
	if (noff != no+nlen) {
	  printf("BAD LEN %d,%d\n", noff, nlen);
	  exit(0);
	}
	xoff += SWS(extract(hdr,16,27));
	if (bm == NULL) {
	  bm = bmp_alloc(bmsz,bmsz);
	}
	bmp_polyline(bm, npts/2, pts, RGB(0xff,0,0x40));
      }
      printf("-- end %.4x\n", xoff);
    }
    off += 4;
  }
  /* Additional Data */
  for (i=0; i<nad; i++) {
    xoff = D(_2b(ptr + off));
    xsz  = SWS(_2b(ptr + off + 2));
    off += 4;
    if (xoff != 0xFFFF)
      printf("  additional:%.4x  size:%d\n", xoff, xsz);
  }
  printf("=========== @roadend\n");
}

/* 7.4 Name Data Frame */
void dumpname(void *ptr, size_t len)
{
  size_t hlen = SWS(_2b(ptr)); // 7.4.1 [SWS] Name Distribution Header
  int na, attr1, attr2,ab,xc,yc,st,off,toff,tnum,k,ang,sx,sy,rx,ry;
  char slen, str[256] = { 0 };
  vector_t v;

  printf("==================== name\n");
  ab = 0;
  for (off=2; off<hlen; off+=4) {
    /* 7.4.1 Name Data Management Information
     *   00 [D] Offset to Name Data List
     *   02 [N] Number of Name Data Records
     */
    toff = D(_2b(ptr + off));
    tnum = _2b(ptr + off + 2);
    if (toff != 0xFFFF) {
      printf("  %2d offset:%.4x  count:%.4x\n", (off-2)/4, toff, tnum);
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

	st = extract(attr1, 8, 10);    // [B] String type
	printf("name: gr:%d str:%d attr:%x [%s] ", extract(na, 0, 11), st, attr2, lookup(types, attr2));
	toff += 6;

	sx = _2b(ptr + toff + 2);
	sy = _2b(ptr + toff + 4);

	rx = extract(sx, 13, 15);
	ry = extract(sy, 13, 15);
	xc = extract(sx, 0, 12) + rx*4096;
	yc = extract(sy, 0, 12) + ry*4096;

	v = matxvec(m,vecinit(xc,yc));
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
	  printf("  string: '%s' x:%d y:%d\n", str, xc, yc);

	  bmp_drawstring(bm, v.v[0],v.v[1], CENTER, CENTER, 0, str, RGB(0xff,0xFF,0xff));
	  toff += slen + 8; // string length
	} else if (st == 4) {
	  /* 7.4.2.1.5 Linear-B 
	  *    00 [B:B:B:B:B:B:N] String Placement Information
	  *    02 [D] Offset to Data
	  *    04 Sequence of String Placement  Records 
	  *    xx String Information Data List
	  */
	  xc = _2b(ptr + toff);
	  yc = _2b(ptr + toff + 2);
	  slen = _2b(ptr + toff + 6)*2;
	  memcpy(str, ptr + toff + 8, slen);
	  printf("  string: '%s' place:%d pts:%d off:%x\n", str, extract(xc, 8, 10), extract(xc, 0, 3), yc);
	  toff += slen + 8;
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
	  ang = _2b(ptr + toff + 6);
	  slen = _2b(ptr + toff + 8)*2;
	  memcpy(str, ptr + toff + 10, slen);
	  printf("  string: '%s' angle:%d x:%d y:%d\n", str, extract(ang, 0, 8), xc, yc);
	  toff += slen + 10;
	} else if (st == 6) {
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
	  slen = _2b(ptr + toff + 8)*2;
	  memcpy(str, ptr + toff + 10, slen);
	  printf("  string: '%s' x:%d y:%d\n", str, xc, yc);

	  bmp_drawstring(bm, v.v[0],v.v[1], CENTER, CENTER, 0, str, RGB(0xff,0xFF,0xff));
	  toff += slen + 10;
	} else {
	  os_dump(ptr + toff, 32);
	  exit(0);
	}
      }
    }
  }
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
  off_t off, poff, goff, boff;
  int i, n, b, p[256], t[256], j, plen, val, ncoord, xc, yc, k, mconst;
  struct mingr_t *gr;
  int *pts;
  static int ns;

  //printf("========= Background %.4x\n", len);
  for (off=2; off<hlen; off+=4) {
    poff = D(_2b(ptr + off));         // 7.3.1.1 [D] element unit offset
    plen = SWS(_2b(ptr + off + 2));   // 7.3.1.1 [SWS] element unit size
    if (poff != 0xFFFF) {
      //printf("  Element: %.4x %.4x [%.4x]\n", poff, plen, poff+plen);

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
	printf("     Shape:%x height:%x #gr:%d offset:%.4x\n",
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
	  printf("      lvl:%2d gr%.3d,%.3d shape:%d #coord:%.4d type:%.4x addl:%.4x rel:%2x,%2x [%s]\n", 
		 extract(lmr->header, 10, 15),
		 i, j, t[i],
		 ncoord, gr->code, gr->addl,
		 extract(gr->sx, 13, 15),
		 extract(gr->sy, 13, 15),
		 lookup(types, gr->code));

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
	    vector_t v;
	    int rx,ry;

	    /* Extract coordinates */
	    rx = extract(gr->sx, 13, 15);           // 0..7
	    ry = extract(gr->sy, 13, 15);           // 0..7
	    xc = extract(gr->sx, 0, 12) + rx*4096;  // 0..8191
	    yc = extract(gr->sy, 0, 12) + ry*4096;  // 0..8191

	    pts = zmalloc(sizeof(int)*2*(ncoord+2));
	    v = matxvec(m,vecinit(xc,yc));
	    pts[0] = v.v[0];
	    pts[1] = v.v[1];

	    //printf("  [%4d,%4d] = [%4d,%4d]\n", xc, yc, pts[0], pts[1]);

	    for (k=0; k<ncoord; k++) {
	      xc += gr->coords[k].xo * mconst;
	      yc += gr->coords[k].yo * mconst;

	      v = matxvec(m,vecinit(xc,yc));
	      pts[k*2+2] = v.v[0];
	      pts[k*2+3] = v.v[1];
	      //printf("  [%4d,%4d] = [%4d,%4d]\n", xc, yc, pts[k*2+2], pts[k*2+3]);
	    }
	    if (drawme) {
	      ns++;
	      if (t[i] == 2) {
		bmp_polyfill(bm, ncoord, pts, typecolor(gr->code));
	      }
	      else {
		bmp_polyline(bm, ncoord, pts, typecolor(gr->code));
	      }
	    }
	    free(pts);
	  }
	  poff += plen;
	}
      }
    }
  }
  printf("== end background\n");
}

int xx[] = { RGB(0xFF,0,0), 0, 
	     RGB(0x0,0xFF,0), 0,
	     RGB(0,0,0xFF), 0,
	     RGB(0xFF,0xFF,0),0,
	     RGB(0xFF,0,0xFF),0,
	     RGB(0x0,0xFF,0xFF),0,
	     RGB(0xFF,0xFF,0xFF),0
};

int typecolor(int type)
{
  switch (type) {
  case 0x121:
  case 0x122:
  case 0x123:
  case 0x124:
    return RGB(0,0,0xFF);
  case 0x128:
    return RGB(0,0xFF,0);
  case 0x131:
    return RGB(0x40,0x40,0x40);
  case 0x132:
    return RGB(0x80,0x80,0x80);
  case 0x134:
    return RGB(0xc0,0xc0,0xc0);
  case 0x140:
  case 0x141:
    return  RGB(0,0xFF,0);
  case 0x210:
  case 0x211:
    return RGB(0xFF,0,0);
  case 0x6180:
    return RGB(0x0,0x80,0x0);
  default:
    return RGB(0xff,0xff,0xff);
  }
  return -1;
}

void showmap(struct lmr_t *lmr, void *map, size_t len)
{
  struct mfde_t *de;
  struct mapframe_t *mf = map;
  int nData, nExt, j, off, i;
  void *dptr;
  int nx, ny, lvl, cx, cy;

  /* Basic map/Extended map numbers */
  lvl = extract(lmr->header, 10, 15);
  nData = extract(lmr->numbers, 12, 15);
  nExt  = extract(lmr->numbers, 8, 11);

  if (drawme) {
    vector_t v;

    v = matxvec(m,vecinit(8192,8192));
    bm = bmp_alloc(bmsz,bmsz);
    bmp_rect(bm,0,0,v.v[0],v.v[1],RGB(0xFF,0xFF,0));
  }
  swapw(&mf->size);
  swapw(&mf->llcode);
  swapw(&mf->dipid);
  swapl(&mf->pmcode);
  swapw(&mf->dsflag);
  swapw(&mf->rlx);
  swapw(&mf->rly);
  swapw(&mf->geo_str);
  swapw(&mf->geo_dec);
  swapl(&mf->rg_addr.addr);
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

  printf("    ==== [%d,%d] lat/lng : @@ lvl:%2d ", cx, cy, lvl);
  showpid(mf->llpid);
  printf(" to ");
  printf("%lf,%lf\n", _my + (_ry-_ly)/ny, _mx + (_rx-_lx)/nx);

  printf("    divint:%d adj:%d type:%d lat:%d lng:%d\n",
	 extract(mf->dipid, 14, 15),
	 extract(mf->dipid, 13, 13),
	 extract(mf->dipid, 8, 9),
	 extract(mf->dipid, 4, 7),
	 extract(mf->dipid, 0, 3));
  if (isin(30.2669, -97.7428, 
	   geo_secs(mf->llpid.lat), geo_secs(mf->llpid.lng), (_rx-_lx)/nx, (_ry-_ly)/ny)) {
    printf(" @@ AUSTIN\n");
  }

  off = 0;
  for (i=0; i<nData; i++) {
    swapl(&de[off].offset);
    swapw(&de[off].size);
    //printf("  Data%d: %.8x %.4x\n", i, D(de[off].offset), SWS(de[off].size));
    if (de[off].size) {
      dptr = map + D(de[off].offset);

      /* 0 == road data, 1 == background data, 2 == name data */
      if (i == 2) {
	dumpname(dptr, SWS(de[off].size));
      }
      if (i == 0) {
	dumproad(dptr, SWS(de[off].size));
      }
      if (i == 1) {
	dumpbkgd(lmr,dptr, SWS(de[off].size));
      }
    }
    off++;
  }
  for (i=0; i<nExt; i++) {
    swapl(&de[off].offset);
    swapw(&de[off].size);
    //printf("  Ext%d: %x %x\n", i, D(de[off].offset), SWS(de[off].size));
    off++;
  }
  if (drawme) {
    bmp_write(bm, "poly.bmp");
    exit(0);
  }
}

/* A = blockset, N = nblocksets
 * B = block,    M = nblocks
 * C = parcel,   O = nparcels
 */
bitmap_t *tb;
void divbsmr(int lvl, int a, int m, int b, int n, int c, int o, int color)
{
  int x, y;

  if (lvl != 0)
    return;
  x = (c%o) + (b%n)*o + (a%m)*n*o;
  y = (c/o) + (b/n)*o + (a/m)*n*o;

  if (tb == NULL)
    tb = bmp_alloc(n*m*o,n*m*o);
  bmp_putpixel(tb, x, y, color);
}

int isin(double y, double x, double ly, double lx, double dy, double dx)
{
  return (y >= ly && y <= (ly+dy) && x >= lx && x <= (lx+dx));
}

void showalldata()
{
  int fd, i, fdp, j, lvl, k, bset, pt, bc, xt, l;
  struct datavol_t dv;
  struct mhr_t *mhr;
  off_t moff, cur;
  void *zdat[34];
  char name[64];
  struct pdmdh_t *pdmdh;
  struct lmr_t *lmr, **lmrmap;
  struct bsmr_t *bsmr, **bsmrmap;

  fd = open("/media/ALLDATA.KWI", O_LARGEFILE|O_RDONLY);
  if (fd < 0)
    return;

  /* Setup matrix */
  m = matxmat(S(bmsz/ssz,bmsz/ssz),I());

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
    swapl(&mhr[i].dsa.addr);
    swapw(&mhr[i].size);
    strncat(name, mhr[i].name, 12);
    printf("-- Record %.2d ", i+1);
    printf("  Addr:%.8" PRIx32 " Size:%.4x  Name:'%s'\n", mhr[i].dsa.addr, mhr[i].size, name);
    if (mhr[i].name[0] == 0 && mhr[i].dsa.addr != -1) {
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
  printgeo(pdmdh->lower_lat, LATITUDE); 
  printf(" ");
  printgeo(pdmdh->left_lng,  LONGITUDE);

  printf("\nupper right:\n");
  printgeo(pdmdh->upper_lat, LATITUDE);
  printf(" ");
  printgeo(pdmdh->right_lng, LONGITUDE);

  _lx = geo_secs(pdmdh->left_lng);
  _rx = geo_secs(pdmdh->right_lng);
  _ly = geo_secs(pdmdh->lower_lat);
  _ry = geo_secs(pdmdh->upper_lat);

  printf("\nlmr_sz: %x  bsmr_sz:%x  bmr_sz:%x nlmr:%x nbsmr:%x filename:%x\n",
         SWS(pdmdh->lmr_sz), pdmdh->bsmr_sz, pdmdh->bmr_sz, pdmdh->nlmr, pdmdh->nbsmr, pdmdh->filename);
  if (isin(30.2669, -97.7428, _ly, _lx, _ry-_ly, _rx-_lx)) {
    printf("@@@ AUSTIN\n");
  }
  for (i=1;i<=64;i<<=1) {
    printf("%2d: nx=%lf  ny=%lf\n",
	   i, (_rx-_lx)/i, (_ry-_ly)/i);
  }

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

    printf("======== lmr%d:  level=%d  upper=%d lower=%d\n", i, 
           extract(lmr->header, 10, 15),
           extract(lmr->header, 4, 7),
           extract(lmr->header, 0, 3));
    for (j=0; j<5; j++) {
      printf(" disp%d: %d\n", 1+j, lmr->dispflag[j]);
    }

    lvl = extract(lmr->header, 10, 15);
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

    /* Extended info */
    xt = _2b(&lmr[1]);
    printf("  #basic map      : %d\n", extract(lmr->numbers, 12, 15));
    printf("  #extend map     : %d\n", extract(lmr->numbers, 8, 11));
    printf("  #basic route    : %d\n", extract(lmr->numbers, 4, 7));
    printf("  #extend route   : %d\n", extract(lmr->numbers, 0, 3));
    printf("  #road           : %d\n", extract(xt, 10, 13)+1); // 7.2 Road Data Frame       1..16
    printf("  #background     : %d\n", extract(xt, 5, 9)+1);   // 7.3 Background Data Frame 1..32
    printf("  #name           : %d\n", extract(xt, 0, 4)+1);   // 7.4 Name Data Frame       1..32

    //os_dump(&lmr[1], SWS(pdmdh->lmr_sz) - sizeof(*lmr));
    moff += SWS(pdmdh->lmr_sz);
  }

  /* 6.1.2 Dump Block Set Management Records */
  bsmrmap = calloc(pdmdh->nbsmr, sizeof(struct bsmr_t *));
  for (i=0; i<pdmdh->nbsmr; i++) {
    bsmr = (struct bsmr_t *)(zdat[0] + moff);
    double lvx,lvy;

    bsmrmap[i] = bsmr;
    swapw(&bsmr->header);
    swapl(&bsmr->bmt_offset);
    swapl(&bsmr->bmt_size);

    lvl = extract(bsmr->header, 10, 15);
    bset = extract(bsmr->header, 0, 7);
    lmr = findlevel(lmrmap, pdmdh->nlmr, lvl);

    bc = 0;
    printf("---- bsmr%d: level=%2d blockset=%3d [%dx%d]\n",
           i, lvl, bset, 1+lmr->nblocksets.lng, 1+lmr->nblocksets.lat);
    if (bsmr->bmt_size) {
      struct bmt_t *bmt;
      off_t boff, poff, toff;
      void *pdat;

      lmr = findlevel(lmrmap, pdmdh->nlmr, lvl);

      boff = 0;
      while (boff < SWS(bsmr->bmt_size)) {
        struct parman_t *pi;
        union mapinfo_t *mi;
	int *ip;

        bc++;  // matches nblocks.lat * nblocks.lng
        bmt = (struct bmt_t *)(zdat[0] + D(bsmr->bmt_offset) + boff);

        swapl(&bmt->dsa.addr);
        swapw(&bmt->size);
        if (bmt->size) {
          poff = getsector(bmt->dsa);
          pdat = zreado(fd, bmt->size * logical_sz, poff, "pdat");

          poff = 0;
          while (poff < bmt->size*logical_sz) {
            pi = (void *)(pdat + poff);

	    /* Parcel Management */
            swapw(&pi->type);
            swapw(&pi->routeoff);
	    if (pi->routeoff != 0xFFFF) {
	      printf("remain bmt:\n");
	      os_dump(pdat + poff, bmt->size * logical_sz - poff);
	      break;
	    }

            pt = extract(pi->type, 8, 9);
            k = (1+lmr->nparcels[pt].lat)*(1+lmr->nparcels[pt].lng);
            printf("  Parcel type %d listtype:%d offset:%x map:%dx%d = %d [%dx%d]\n", 
                   extract(pi->type, 8, 9),
                   extract(pi->type, 0, 7),
                   D(pi->routeoff),
                   1+lmr->nparcels[pt].lat,
                   1+lmr->nparcels[pt].lng,
                   k,
		   (1+lmr->nblocksets.lng)*(1+lmr->nblocks.lng)*(1+lmr->nparcels[pt].lng),
		   (1+lmr->nblocksets.lat)*(1+lmr->nblocks.lat)*(1+lmr->nparcels[pt].lat));
            poff += 4;

	    /* Calculate integrated packets */
	    ip = zmalloc(sizeof(int)*k);
	    for (j=0; j<k; j++) {
	      uint32_t add,size,l;

	      add  = _4b(pdat + poff + j*6);
	      size = _2b(pdat + poff + j*6 + 4);
	      if (!ip[j] && add != 0xffffffff && size) {
		ip[j] =  j+1;
		for (l=j+1; l<k; l++) {
		  /* If address is same, both are in same integrated packet */
		  if (add == _4b(pdat + poff + l*6)) {
		    ip[l] = j+1;
		  }
		}
	      }
	      else if (!ip[j] && add != 0xffffffff && !size) {
		/* This is a divided packet */
		ip[j] = -1;
	      }
	      if (add != 0xFFFFFFFF) {
		printf("   [%3d,%3d] lvl:%d.%d Add: %.8x  Size:%.4x ip:%d\n", 
		       j % (1+lmr->nparcels[pt].lat), 
		       j / (1+lmr->nparcels[pt].lat), 
		       lvl, pt, add, size, ip[j]);
	      }
	    }

            for (j=0; j<k; j++) {
              void *mdat;
              off_t mapoff;

	      /* lvl=10, 3 == Hawaii
	       * lvl=10, 6 == Cali
	       * lvl=10: [2,0]
	       * lvl=4:  [34,26]: 1699/1763 ok
	       * lvl=2:  [10,42]: 2569
	       * lvl=0:  [40,44]: 2857
	       */
	      drawme = (ip[j] == 1763 && lvl == 4);
              if (poff > bmt->size * logical_sz) {
		fprintf(stderr,"boooo\n");
                break;
	      }
              mi = (void *)(pdat + poff);
              swapl(&mi->map0.dsa.addr);
              swapw(&mi->map0.size);

              if (mi->map0.size) {
                printf("    MapPar Addr: level:%d.%d  blockset:%d/%d block:%d/%d parcel:%d/%d\n",
                       lvl, pt, 
                       bset, (1+lmr->nblocksets.lng) * (1+lmr->nblocksets.lat),
                       bc-1, (1+lmr->nblocks.lng) * (1+lmr->nblocks.lat),
                       j,   k);
		if (j+1 == ip[j] && drawme) {
		  mapoff = getsector(mi->map0.dsa);
		  mdat = zreado(fd, mi->map0.size * logical_sz, mapoff, "map");
		  showmap(lmr, mdat, mi->map0.size * logical_sz);
		  zfree(mdat, mi->map0.size * logical_sz);
		}
              } else if (mi->map0.dsa.addr != -1) {
                printf("    MapPar Addr: level:%d.%d  blockset:%d/%d block:%d/%d parcel:%d/%d  Ref :%x\n",
                       lvl, pt, 
                       bset, (1+lmr->nblocksets.lng)*(1+lmr->nblocksets.lat),
                       bc-1, (1+lmr->nblocks.lng)*(1+lmr->nblocks.lat),
                       j, k,
                       D(mi->map0.dsa.addr));
              }
              poff += 6;
            }
	    zfree(ip,sizeof(int)*k);
          }
        }
        boff += 6;
      }
      /* ASSERT: Number of blocks per blockset == nbx[lvl] * nby[lvl] */
      //printf("BlockSet Count %d = %d\n", lvl, bc);
    }

    moff += sizeof(*bsmr);
  }
  /* ASSERT: count of blocksets per level == nbsx[lvl] * nbsy[lvl] */
  fprintf(stderr,"done\n");
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

