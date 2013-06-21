#define _LARGEFILE64_SOURCE
#define _LARGEFILE_SOURCE
#define _FILE_OFFSET_BITS 64
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <inttypes.h>
#include <time.h>
#include <endian.h>
#include <string.h>
//#include <oslib.h>
#include "bmp.h"

uint16_t _2b(void *v)
{
  return be16toh(*(uint16_t *)v);
}

uint32_t _3b(void *v)
{
  uint8_t  *s = v;

  return ((s[0] << 16) + (s[1] << 8) + s[2]);
}

uint32_t _4b(void *v)
{
  return be32toh(*(uint32_t *)v);
}

static void os_dump(void *b, int len)
{
  unsigned char *bb = (unsigned char *)b;
  int i,j,c;
  
  for(i=0;i<len;i+=16) {
    printf("%.4x: ", i);
    for(j=0;j<16;j++) {
      if (i+j >= len)
	printf("XX ");
      else
	printf("%.02x ", bb[i+j]);
    }
    printf("  ");
    for(j=0;j<16;j++) {
      c = bb[i+j];
      if (i+j >= len || c < ' ' || c > 'z')
	c = '.';
      printf("%c",c);
    }
    printf("\n");
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

/* ssssssss ssssssss ssssssss DLnnnnnn */
struct sectoraddress
{
  uint32_t addr;
}  __attribute__((packed));

/* Fsssssss ssssssss sssssfff */
#define LATITUDE   "NS"
#define LONGITUDE  "EW"
#define LL_DIR     (1L << 23)

typedef uint8_t geonum_t[3];

void printgeo(geonum_t geo, const char *ll)
{
  uint32_t v,flag,secs;

  v = _3b(geo);
  flag = !!(v & LL_DIR);
  secs = v & ~LL_DIR;

  printf("%c%d %d %g", ll[flag], 
	 secs / (3600 * 8), 
	 ((secs / ( 60 * 8)) % 60),
	 (secs % (60 * 8)) / 8.0);
}

struct pid_t
{
  geonum_t lat;
  uint8_t  exp1;
  geonum_t lng;
  uint8_t  exp2;
}  __attribute__((packed));

struct mid_t
{
  struct pid_t	loc;
  uint8_t		floor;
  uint8_t		reserved;
  uint16_t	date;
} __attribute__((packed));

struct lmm_t
{
  uint16_t 	n;
  uint16_t 	dummy;
} __attribute__((packed));

struct sii_t
{
  struct mid_t 	mid;
  uint16_t 	nm;
  uint16_t 	dummy;
} __attribute__((packed));

struct mii_t
{
  uint8_t 		category;
  uint8_t 		reserved[3];
  uint8_t 		modulename[52];
  uint8_t 		modulever[8];
} __attribute__((packed));

struct mmi_t 	
{
  uint16_t 		date_start;
  uint16_t 		date_end;
  uint8_t  		title[64];
  uint8_t  		mandep[182];
  uint32_t 		addr;
  uint16_t 		size;
} __attribute__((packed)); 

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
 * datavol
 * mht[n]
 *
 * datavol_t {
 *   mid_t systemspec_mid;
 *   char  systemspec_ssi[52];
 *   mid_t data_mod;
 *   char  data_ssi[52];
 *   mid_t system_mid;
 *   char  system_ssi[52];
 *   char  format_ver[64];
 *   char  data_ver[64];
 *   char  disk_title[128];
 *   ad_dai_t
 *   ad_sid_t
 */
struct datavol_t
{
  struct mid_t	spec_mid;
  char		spec_ssi[52];
  struct mid_t	data_mid;
  char		data_ssi[52];
  struct mid_t	system_mid;
  char		system_ssi[20];

  char		format_ver[64];
  char		data_ver[64];
  char		disk_title[128];
  uint16_t	        contents[4];
  char		media_version[32];

  /* Data coverage */
  struct pid_t	box_ll;
  struct pid_t	box_ur;

  uint16_t	log_size;		// logical sector size
  uint16_t	sector_size;		// sector size
  uint16_t	background;		// background data default information

  uint8_t	res1[14];
  uint8_t	level_mgmt[256];	// level management information
  uint8_t	res2[1300];
} __attribute__((packed));

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
  printgeo(mid.loc.lat, LATITUDE);
  printf(" ");
  printgeo(mid.loc.lng, LONGITUDE);
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

void *zreado(int fd, int sz, off_t off)
{
  void *ptr;

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

    mbuf = zreado(fd, dde[i].length, dde[i].offset*2 + 0x800);
    os_dump(mbuf, dde[i].length);
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
} __attribute__((packed));

/* Parcel Data Management Distribution Header */
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

  uint16_t		lmr_sz;         // [SWS] size in words
  uint16_t		bsmr_sz;        // [SWS] size in words
  uint16_t		bmr_sz;         // [SWS] size in words

  uint16_t		nlmr;          // [N]
  uint16_t		nbsmr;         // [N]

  /* lmr_t        lmr[var];    Level Management Records
   * bsmr_t       bsmr[var];   Block Set Management Records
   * bmt_t        bmt[var];    Block Management Tables
   */
} __attribute__((packed));

/* Level Management Record */
struct lmr_t
{
  uint16_t		header;       // [I:N]
  uint16_t		numbers;      // [N:N:N:N]
  uint32_t		dispflag[5];  // [N]
  struct {
    uint8_t             lat;
    uint8_t             lng;
  } nblocksets;
  struct {
    uint8_t             lat;
    uint8_t             lng;
  } nblocks;
  struct {
    uint8_t             lat;
    uint8_t             lng;
  } nparcels[3];                      // 0 = normal, 1=pardiv1, 2=pardiv2, 3=pardiv3
  uint16_t		bsmr_off;     // [D] offset in words from zdat[0]
  uint16_t		nrsize;       // [SWS]
} __attribute__((packed));

/* Block Set Management Record */
struct bsmr_t
{
  uint16_t		header;        // [I:N] header
  uint32_t		bmt_offset;    // [D]   offset in words from zdat[0]
  uint32_t		bmt_size;      // [SWS] size in words
} __attribute__((packed));

/* Reference Record of Parcel Management Information */
struct bmt_t
{
  struct sectoraddress  dsa;           // [DSA] sector address
  uint16_t              size;          // [BS]  size in logical sectors
  uint8_t               filename[12];  // [C]   optional
} __attribute__((packed));

/* Parcel Management Info */
struct parman_t
{
  uint16_t             type;       // [N:N] Parcel Management type
  uint16_t             routeoff;   // [D] route guidance parcel management list
};

/* Main Map Parcel Management Record */
union mapinfo_t {
  struct {
    struct sectoraddress dsa;      // [DSA] sector address
    uint16_t             size;    // [BS] size in logical sectors
  } __attribute__((packed)) map0;
  struct {
    struct sectoraddress dsa;
    uint16_t             size1;
    uint16_t             size2;
  } __attribute__((packed)) map1;
  struct {
    struct sectoraddress dsa;
    uint16_t             size1;
    uint16_t             size2;
  } __attribute__((packed)) map2;
  struct {
    struct sectoraddress dsa;
    uint16_t             size;
    char                 filename[12];
  } __attribute__((packed)) map100;
} __attribute__((packed));

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

  if (bits & (1L << (end - start)))
    bits |= (-1L & ~mask);
  return bits;
}

uint32_t extract(uint32_t val, int start, int end)
{
  uint32_t mask = (1L << (end - start + 1)) - 1;
  return (val >> start) & mask;
}

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
} __attribute__((packed));

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
} __attribute__((packed));

union strinfo_t
{
  struct {
    uint16_t bkinfo;
    uint16_t nx;
    uint16_t ny;
  } __attribute__((packed)) barystr;
  struct {
    uint16_t addinfo;
    uint16_t nx;
    uint16_t ny;
    uint16_t loc;
    char     str[1];
  } __attribute__((packed)) symbstr;
} __attribute__((packed));

void dumpname(void *ptr, size_t len)
{
  size_t hlen = _2b(ptr) * 2;
  int na, attr1, attr2,ab,xc,yc,st;

  /* [SWS] HeaderSize
   * [VAR] Name Data Lists
   * [VAR] Extended Data 
   */

  /* [W] Name Data Header
   * [W] Attr1
   * [W] Attr2
   * 
   * u16 [B:B:B] AddBkgInfo
   * u16 [N:NX]  X-Coord
   * u16 [N:NX]  Y-Coord
   */
  na = _2b(ptr + hlen);
  attr1 = _2b(ptr + hlen + 2);
  attr2 = _2b(ptr + hlen + 4);
  st = extract(attr1, 8, 10);
  printf("name: gr:%d str:%d attr:%x\n", extract(na, 0, 11), st, attr2);
  if (st == 6) {
    char *halign[] = { "right", "left", "middle", "rsv" };
    char *valign[] = { "above", "under", "left", "right" };
    int loc, clen;
    char str[256] = { 0 };

    ab = _2b(ptr + hlen + 6);
    xc = _2b(ptr + hlen + 8);
    yc = _2b(ptr + hlen +10);
    loc = _2b(ptr + hlen+12);
    clen = _2b(ptr + hlen+14);
    memcpy(str, ptr + hlen + 16, clen*2);
    printf("      xc:%d rel:%d   yc:%d  rel:%d  h:%s  v:%s  [%s]\n",
	   extract(xc, 0, 12), extract(xc, 13, 15),
	   extract(yc, 0, 12), extract(yc, 13, 15),
	   halign[extract(loc, 14, 15)], valign[extract(loc, 12, 13)],
	   str);
  }
  os_dump(ptr, hlen);
  os_dump(ptr+hlen, min(len-hlen,32));
}

void dumpbkg(void *ptr, size_t len)
{
  size_t hlen = _2b(ptr) * 2;
  off_t poff, goff;
  int i, n, b, p, j;

  poff = hlen;
  n = _2b(ptr + poff);

  printf("# Background: %d\n", n);
  //os_dump(ptr + poff, 32);

  poff += 2;
  for (i=0; i<n; i++) {
    b = _2b(ptr + poff + 2);
    printf(" Offset: %x\n", _2b(ptr + poff)*2);
    printf(" Shape : %d\n", extract(b, 14, 15));
    printf(" Height: %d\n", extract(b, 13, 13));
    printf(" #Graph: %d\n", extract(b, 0, 11));

    goff = _2b(ptr + poff)*2 + hlen;
    p = extract(b, 0, 11);
    for (j=0; j<p; j++) {
      printf("  gr%d: hdr:%x scale:%x type:%x addl:%x\n",
	     j, _2b(ptr + goff), _2b(ptr + goff + 2), _2b(ptr + goff + 4), _2b(ptr + goff + 6));
      break;
    }
    poff += 4;
  }    
}

void showmap(struct lmr_t *lmr, void *map, size_t len)
{
  struct mfde_t *de;
  struct mapframe_t *mf = map;
  int nData, nExt, j, off, i;
  void *dptr;

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
  swapl(&mf->rg_addr.addr);
  swapw(&mf->rg_size);
  swapw(&mf->nregion);

  de = (void *)&mf[1] + mf->nregion * 4;

  printf(" @@@@ ");
  showpid(mf->llpid);
  printf("\n");

  /* [N] Map Distribution Header
   * [VAR] Main Map Basic Data Frame
   * [VAR] Main Map Extended Data Frame
   */
  printf("  size    :  %x\n", mf->size * 2);
  printf("  coord   : [%d,%d]\n", extract(mf->llcode, 0, 7), extract(mf->llcode, 8, 15));
  printf("  divintid: %d\n", extract(mf->dipid, 14, 15));
  printf("  adjflag : %d\n", extract(mf->dipid, 13, 13));
  printf("  type#   : %d\n", extract(mf->dipid, 8, 9));
  printf("  relx.y  : %d, %d\n", 
	 extract(mf->dipid, 4, 7), extract(mf->dipid, 0, 3));
  printf("  area#   : %d\n", extract(mf->pmcode, 24, 31));
  printf("  rgoff   : %x\n", mf->rg_addr.addr);
  printf("  rgsize  : %d\n", mf->rg_size);
  printf("  nroute  : %d\n", mf->nregion);
  //os_dump(map,mf->size * 2);
  
  off = 0;
  for (i=0; i<nData; i++) {
    swapl(&de[off].offset);
    swapw(&de[off].size);
    printf("  Data%d: %lx %x\n", i, de[off].offset*2, de[off].size*2);
    if (de[off].size) {
      dptr = map + de[off].offset*2;
      //os_dump(map + de[off].offset*2, de[off].size*2);

      if (i == 2) {
	/* Background data */
	dumpname(dptr, de[off].size*2);
      }
    }
    off++;
  }
  for (i=0; i<nExt; i++) {
    swapl(&de[off].offset);
    swapw(&de[off].size);
    printf("  Ext%d: %lx %x\n", i, de[off].offset*2, de[off].size*2);
    off++;
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

#if 0
void showparcel(void *pdat, int poff)
{
  struct parman_t *pi;
  union mapinfo_t *mi;
  int pt, pl, nparcels, i;

  pi = pdat + poff;
  
  swapw(&pi->type);
  swapw(&pi->routeoff);

  pt = extract(pi->type, 8, 9);  // parcel type
  pl = extract(pi->type, 0, 7);  // parcel list type
  
  poff += 4;
  nparcels = (1+lmr->nparcels[pt].lat)*(1+lmr->nparcels[pt].lng);
  for (i=0; i<nparcels; i++) {
    mi = pdat + poff;
    swapl(&mi->map0.dsa.addr);
    swapw(&mi->map0.size);

    if (mi->map0.size) {
      /* Map node */
    } else if (mi->map0.dsa.addr != -1) {
      /* Reference node */
      showparcel(pdat, mi->map0.dsa.addr * 2);
    }
    poff += 6;
  }
}
#endif

void showalldata()
{
  int fd, i, fdp, j, lvl, k, bset, pt, bc;
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
      zdat[i] = zreado(fd, mhr[i].size * logical_sz, moff);
      //os_dump(zdat[i], mhr[i].size * logical_sz);
    }
  }
  /* Dump PDMR */
  moff = sizeof(struct pdmdh_t);
  pdmdh = zdat[0];
  swapw(&pdmdh->size);
  swapw(&pdmdh->lmr_sz);
  swapw(&pdmdh->bsmr_sz);
  swapw(&pdmdh->bmr_sz);
  swapw(&pdmdh->nlmr);
  swapw(&pdmdh->nbsmr);
  swapw(&pdmdh->filename);
  printf("lower left:\n");
  printgeo(pdmdh->lower_lat, LATITUDE);	
  printgeo(pdmdh->left_lng,  LONGITUDE);	

  printf("upper right:\n");
  printgeo(pdmdh->upper_lat, LATITUDE);	
  printgeo(pdmdh->right_lng, LONGITUDE);	
  printf("lmr_sz: %x  bsmr_sz:%x  bmr_sz:%x nlmr:%x nbsmr:%x filename:%x\n",
	 pdmdh->lmr_sz*2, pdmdh->bsmr_sz, pdmdh->bmr_sz, pdmdh->nlmr, pdmdh->nbsmr, pdmdh->filename);

  /* Dump level records */
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

    printf("======== lmr%d:  level=%d  upper=%d lower=%d moff=%d\n", i, 
	   extract(lmr->header, 10, 15),
	   extract(lmr->header, 4, 7),
	   extract(lmr->header, 0, 3), 
	   moff);
    printf("  #basic map         : %d\n", extract(lmr->numbers, 12, 15));
    printf("  #extend map        : %d\n", extract(lmr->numbers, 8, 11));
    printf("  #basic route       : %d\n", extract(lmr->numbers, 4, 7));
    printf("  #extend route      : %d\n", extract(lmr->numbers, 0, 3));
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
    printf(" bsmroff:          : %d\n",
	   lmr->bsmr_off * 2);
    printf(" node record size  : %d\n", lmr->nrsize * 2);
    moff += pdmdh->lmr_sz * 2;
  }

  /* Dump Block Set Management Records */
  bsmrmap = calloc(pdmdh->nbsmr, sizeof(struct bsmr_t *));
  for (i=0; i<pdmdh->nbsmr; i++) {
    bsmr = (struct bsmr_t *)(zdat[0] + moff);

    bsmrmap[i] = bsmr;
    swapw(&bsmr->header);
    swapl(&bsmr->bmt_offset);
    swapl(&bsmr->bmt_size);

    lvl = extract(bsmr->header, 10, 15);
    bset = extract(bsmr->header, 0, 7);
    lmr = findlevel(lmrmap, pdmdh->nlmr, lvl);

    bc = 0;
    printf("---- bsmr%d: level=%2d blockset=%3d moff=%d [%dx%d]\n",
	   i, lvl, bset, moff, 1+lmr->nblocksets.lng, 1+lmr->nblocksets.lat);
    if (bsmr->bmt_size) {
      struct bmt_t *bmt;
      off_t boff, poff;
      void *pdat;

      lmr = findlevel(lmrmap, pdmdh->nlmr, lvl);
      //os_dump(zdat[0] + bsmr->bmt_offset * 2, bsmr->bmt_size * 2);
      printf("  bmt_offset: %d\n", bsmr->bmt_offset * 2);
      printf("  bmt_size  : %d\n", bsmr->bmt_size * 2);

      boff = 0;
      while (boff < bsmr->bmt_size * 2) {
	struct parman_t *pi;
	union mapinfo_t *mi;

	bc++;  // matches nblocks.lat * nblocks.lng
	bmt = (struct bmt_t *)(zdat[0] + bsmr->bmt_offset*2 + boff);

	swapl(&bmt->dsa.addr);
	swapw(&bmt->size);
	if (bmt->size) {
	  poff = getsector(bmt->dsa);
	  printf("   Parcel addr: %lx  size:%ld  off=%d\n", bmt->dsa.addr, bmt->size * logical_sz, poff);
	  pdat = zreado(fd, bmt->size * logical_sz, poff);
	  //os_dump(pdat, bmt->size * logical_sz);

	  poff = 0;
	  while (!poff && poff < bmt->size*logical_sz) {
	    pi = (void *)(pdat + poff);

	    swapw(&pi->type);
	    swapw(&pi->routeoff);

	    pt = extract(pi->type, 8, 9);
	    printf("   ===================== %x\n", poff);
	    printf("   Parcel type     : %d\n", extract(pi->type, 8, 9));
	    printf("   Parcel list type: %d\n", extract(pi->type, 0, 7));
	    printf("   Route Offset    : %d\n", pi->routeoff);

	    k = (1+lmr->nparcels[pt].lat)*(1+lmr->nparcels[pt].lng);
	    printf("   Map count       : %dx%d = %d\n", 
		   1+lmr->nparcels[pt].lat, 1+lmr->nparcels[pt].lng, k);

	    poff += 4;
	    for (j=0; j<k; j++) {
	      void *mdat;
	      off_t mapoff;

	      if (poff > bmt->size * logical_sz)
		break;
	      mi = (void *)(pdat + poff);
	      swapl(&mi->map0.dsa.addr);
	      swapw(&mi->map0.size);

	      if (mi->map0.size) {
		printf("    MapPar Addr: level:%d.%d  blockset:%d/%d block:%d/%d parcel:%d/%d  Addr:%lx  size:%lx\n", 
		       lvl, pt, 
		       bset, (1+lmr->nblocksets.lng) * (1+lmr->nblocksets.lat),
		       bc-1, (1+lmr->nblocks.lng) * (1+lmr->nblocks.lat),
		       j,   k,
		       mi->map0.dsa.addr,
		       mi->map0.size * logical_sz);
#if 0
		if (pt == 0)
		  divbsmr(lvl, bset, 1+lmr->nblocksets.lng, bc-1, 1+lmr->nblocks.lng, j, 1+lmr->nparcels[0].lng, RGB(0,0xFF,0));
#endif
		mapoff = getsector(mi->map0.dsa);
		mdat = zreado(fd, mi->map0.size * logical_sz, mapoff);
		showmap(lmr, mdat, mi->map0.size * logical_sz);
		free(mdat);
	      } else if (mi->map0.dsa.addr != -1) {
		printf("    MapPar Addr: level:%d.%d  blockset:%d/%d block:%d/%d parcel:%d/%d  Ref :%lx\n",
		       lvl, pt, 
		       bset, (1+lmr->nblocksets.lng)*(1+lmr->nblocksets.lat),
		       bc-1, (1+lmr->nblocks.lng)*(1+lmr->nblocks.lat),
		       j, k,
		       mi->map0.dsa.addr * 2);
#if 0
		if (pt == 0)
		  divbsmr(lvl, bset, 1+lmr->nblocksets.lng, bc-1, 1+lmr->nblocks.lng, j, 1+lmr->nparcels[0].lng, RGB(0xFF,0,0));
#endif
	      }
	      poff += 6;
	    }
	  }
	}
	boff += 6;
      }
      /* ASSERT: Number of blocks per blockset == nbx[lvl] * nby[lvl] */
      printf("BlockSet Count %d = %d\n", lvl, bc);
    }

    moff += sizeof(*bsmr);
  }
  if (tb)
    bmp_write(tb, "kiwi.bmp");
  /* ASSERT: count of blocksets per level == nbsx[lvl] * nbsy[lvl] */
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

