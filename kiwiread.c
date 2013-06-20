#define _LARGEFILE64_SOURCE
#define _LARGEFILE_SOURCE
#define _FILE_OFFSET_BITS 64

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <inttypes.h>
#include <oslib.h>
#include <endian.h>
#include <math.h>

typedef uint8_t _3bgeo[3];

uint16_t _2b(void *v)
{
	uint8_t *s = v;

	return ((s[0] << 8) + s[1]);
}

uint32_t _3b(void *v)
{
  uint8_t *s = v;

  return ((s[0] << 16) + (s[1] << 8) + s[2]);
}

uint32_t _4b(void *v)
{
  return be32toh(*(uint32_t *)v);
}

double _deg(void *v)
{
  uint32_t deg = _3b(v);

  return (deg & 0xFFFFFF) / 8.0;
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

/* ssssssss ssssssss ssssssss DLnnnnnn 
 *   s = sector address
 *   D = Disk Fide flag (0=SideA, 1=SideB)
 *   L = Storage Layer Flag (0=single, 1=double)
 *   n = # of logical sectors
 */
struct sectoraddress_t
{
  uint32_t addr;
}  __attribute__((packed));

void printsa(struct sectoraddress_t sa)
{
  printf("Sector Address: %c%c%d nlogical:%d\n",
	 sa.addr & 0x80 ? 'B' : 'A',
	 sa.addr & 0x40 ? 'D' : 'S',
	 sa.addr >> 8,
	 sa.addr & 0x3F);
}

/* Fsssssss ssssssss sssssfff -------- 
 *   F = N/S or E/W flag (0=N/E, 1=S/W)
 *   s = latitude seconds
 *   f = latitude in 1/8 seconds
 */
struct geonum_t
{
  _3bgeo      secs;
}  __attribute__((packed));


#define LATITUDE  "NS"
#define LONGITUDE "EW"
#define LL_DIR    (1L << 23)

void printgeo(struct geonum_t num, const char *tag)
{
  uint32_t v = _3b(&num.secs);
  double tot,sec;
  int deg,mn,flag;

  flag = !!(v & LL_DIR);
  sec  = (v & ~LL_DIR) / 8.0;
  deg  = sec / 3600;
  mn   = ((int)(sec / 60)) % 60;
  sec  = fmod(sec, 60.0);

  printf("%c%d %d %g", tag[flag], deg, mn, sec);
}

/* Parcel ID */
struct pid_t
{
  struct geonum_t lat;
  uint8_t         r1;
  struct geonum_t lng;
  uint8_t         r2;
}  __attribute__((packed));

/* Manufacturer ID */
struct mid_t
{
  struct pid_t	loc;
  uint8_t	floor;          // building floor
  uint8_t	reserved;
  uint16_t	date;           // number of days from 1/1/1997
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
 *   mid_t         systemspec_mid;
 *   char          systemspec_ssi[52];
 *   mid_t         data_mod;
 *   char          data_ssi[52];
 *   mid_t         system_mid;
 *   char          system_ssi[52];
 *   char          format_ver[64];
 *   char          data_ver[64];
 *   char          disk_title[128];
 *   ad_dai_t
 *   ad_sid_t
 */
struct datavol_t
{
  /* System Specific Identification */
  struct mid_t	spec_mid;
  char		spec_ssi[52];

  /* Data author identification */
  struct mid_t	data_mid;
  char		data_ssi[52];

  /* System identification */
  struct mid_t	system_mid;
  char		system_ssi[20];

  char		format_ver[64];			/* Format version number */
  char		data_ver[64];			/* Data Version Number */
  char		disk_title[128];		/* Disk Title */
  uint8_t		contents[8];			/* Data Contents */
  char		media_version[32];		/* Media Version Number */
  struct pid_t	box_ll;				/* LL box */
  struct pid_t	box_ur;				/* UR box */
  uint16_t	log_size;			/* Logical Sector Size */
  uint16_t	sector_size;			/* Sector Size */
  uint16_t	background;			
  uint8_t		res1[14];
  uint8_t		level_mgmt[256];
  uint8_t		res2[1300];
} __attribute__((packed));

off_t getsector(struct sectoraddress_t sa)
{
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
    free(mbuf);
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


/* Management Header Table record */
struct mhr_t
{
  struct sectoraddress_t 	dsa;
  uint16_t		size;
  char			name[12];
} __attribute__((packed));

/* Parcel Data Management Distribution Header */
struct pdmdh_t
{
  uint16_t		size;          // SWS
  uint16_t		res1;
  uint16_t		filename;      // :B
  uint16_t		res2;
  struct geonum_t	upper_lat;     // B:N
  struct geonum_t	lower_lat;     // B:N
  struct geonum_t	left_lng;      // B:N
  struct geonum_t	right_lng;     // B:N
  uint16_t		lmr_sz;        // SWS == 0x20
  uint16_t		bsmr_sz;       // SWS == 0x5
  uint16_t		bmr_sz;        // SWS
  uint16_t		nlmr;          // N2
  uint16_t		nbsmr;         // N2
  // level management records[nlmr]
  // block set management records[nbsmr]
  // block management tables[nbsmr]
} __attribute__((packed));

struct lmr_t
{
	uint16_t		header;    // levelnumber|reserved|numUpper|numLower
	uint16_t		numbers;
	uint32_t		dispflag[5];
	uint16_t		nblocksets;
	uint16_t		nblocks;
	uint16_t		nparcels;
	uint16_t		ndivpar1;
	uint16_t		ndivpar2;
	uint16_t		ndivpar3;
	uint16_t		bsmr_off;
	uint16_t		nrsize;

	/* Expansion field */
} __attribute__((packed));

struct bsmr_t
{
  uint16_t		header;
  uint32_t		bmt_offset;
  uint32_t		bmt_size;
} __attribute__((packed));

struct bmt_t
{
} __attribute__((packed));


void showalldata()
{
	int fd, i, j, fdp;
  struct datavol_t dv;
  struct mhr_t *mhr;
  off_t moff, cur;
  void *zdat[34];
  char name[64];
  struct pdmdh_t *pdmdh;
  struct lmr_t *lmr;

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

  sector_sz = dv.sector_size;
  logical_sz = dv.log_size;

  printf("system specific: %s\n", dv.spec_ssi);
  showmid(dv.spec_mid);
  printf("data author: %s\n", dv.data_ssi);
  showmid(dv.data_mid);
  printf("system: %s\n", dv.system_ssi);
  showmid(dv.system_mid);
  printf("fmt:%s\ndata:%s\ntitle:%s\nmedia_ver:%s\n", dv.format_ver, dv.data_ver, dv.disk_title, dv.media_version);
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

  /* Management Header Table
   * 
   * Record 1: PRDM Parcel-related Data Management
   * Record 2: RRDM Region-related Data Management
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
    printf("  Addr:%.8" PRIx32 " Size:%.4x  Name:'%s'\n", mhr[i].dsa, mhr[i].size, name);
    if (mhr[i].name[0] == 0 && mhr[i].dsa.addr != -1) {
      moff = getsector(mhr[i].dsa);
      printf("off: %llx\n", moff);
      zdat[i] = zreado(fd, mhr[i].size * logical_sz, moff);
      //os_dump(zdat[i], mhr[i].size * logical_sz);
    }
  }

  /* Parcel Data Management Header
   * 
   * uint16_t size
   * uint16_t reserved
   * uint16_t filename designation
   * uint16_t reserved
   * _3bdeg   latu;
   * _3bdeg   latl;
   * _3bdeg   lngl;
   * _3bdeg   lngr;
   * uint16_t size of level management record in words [lmr_sz]
   * uint16_t size of block set mangement record in words [bsmr_sz]
   * uint16_t size of block management record in words [bmr_sz]
   * uint16_t number of level management records [lmr_num]
   * uint16_t number of block set management records [bsmr_num]
   * uint8_t  level management records [size = lmr_sz * lmr_num]
   * uint8_t  block set management records [size = bsmr_sz * bsmr_num]
   */
  printf("===================== PDMR\n");
  os_dump(zdat[0], 128);
  pdmdh = zdat[0];	
  swapw(&pdmdh->size);
  swapw(&pdmdh->lmr_sz);
  swapw(&pdmdh->bsmr_sz);
  swapw(&pdmdh->bmr_sz);
  swapw(&pdmdh->nlmr);
  swapw(&pdmdh->nbsmr);
  printf("Upper left:\n");
  printgeo(pdmdh->upper_lat, LATITUDE);
  printf("  ");
  printgeo(pdmdh->left_lng, LONGITUDE);	
  printf("\nLower right\n");
  printgeo(pdmdh->lower_lat, LATITUDE);	
  printf("  ");
  printgeo(pdmdh->right_lng, LONGITUDE);	
  printf("\n");
  printf("LMR size = %d\n", pdmdh->lmr_sz);
  printf("BSMR size = %d\n", pdmdh->bsmr_sz);
  printf("BMR size = %d\n", pdmdh->bmr_sz);
  printf("LMR num = %d\n", pdmdh->nlmr);
  printf("BSMR num = %d\n", pdmdh->nbsmr);

  moff = sizeof(*pdmdh);
  os_dump(zdat[0] + moff, 32);
  for (i=0; i<pdmdh->nlmr; i++) {
	  char *pParcels[] = { "1x1", "2x2", "4x4", "8x8", "16x16", "32x32" };
	  
    lmr = (struct lmr_t *)(zdat[0] + moff);
    moff += pdmdh->lmr_sz * 2;
    swapw(&lmr->header);
    swapw(&lmr->numbers);
    swapl(&lmr->dispflag[0]);
    swapl(&lmr->dispflag[1]);
    swapl(&lmr->dispflag[2]);
    swapl(&lmr->dispflag[3]);
    swapl(&lmr->dispflag[4]);
    swapw(&lmr->nblocksets);
    swapw(&lmr->nblocks);
    swapw(&lmr->nparcels);
    swapw(&lmr->ndivpar1);
    swapw(&lmr->ndivpar2);
    swapw(&lmr->ndivpar3);
    swapw(&lmr->bsmr_off);
    swapw(&lmr->nrsize);
    printf("-------- lmr%d: %x off:%d\n", i, lmr->header, moff);
    printf("  level number: %d  upper:%s lower:%s\n", 
	   lmr->header >> 10, pParcels[(lmr->header >> 4)  & 0xF],
	   pParcels[lmr->header & 0xF]);
    printf("  nbasic map: %d\n", (lmr->numbers >> 12) & 0xF);
    printf("  nextend map:%d\n", (lmr->numbers >> 8)  & 0xF);
    printf("  nbasic rte: %d\n", (lmr->numbers >> 4) & 0xF);
    printf("  nextend rte:%d\n", (lmr->numbers >> 0) & 0xF);
    for (j=0; j<5; j++) {
	    printf("   dispscale%d:  %d\n", j+1, lmr->dispflag[j]);
    }
    printf("  #latblockset: %d\n", 1+ ((lmr->nblocksets >> 8) & 0xFF));
    printf("  #lngblockset: %d\n", 1+ ((lmr->nblocksets >> 0) & 0xFF));

    printf("  #latblocks  : %d\n", 1+ ((lmr->nblocks >> 8) & 0xFF));
    printf("  #lngblocks  : %d\n", 1+ ((lmr->nblocks >> 0) & 0xFF));

    printf("  #latparcels : %d\n", 1+ ((lmr->nparcels >> 8) & 0xFF));
    printf("  #lngparcels : %d\n", 1+ ((lmr->nparcels >> 0) & 0xFF));

    printf("  #latparcel1 : %d\n", 1+ ((lmr->ndivpar1 >> 8) & 0xFF));
    printf("  #lngparcel1 : %d\n", 1+ ((lmr->ndivpar1 >> 0) & 0xFF));

    printf("  #latparcel2 : %d\n", 1+ ((lmr->ndivpar2 >> 8) & 0xFF));
    printf("  #lngparcel2 : %d\n", 1+ ((lmr->ndivpar2 >> 0) & 0xFF));

    printf("  #latparcel3 : %d\n", 1+ ((lmr->ndivpar3 >> 8) & 0xFF));
    printf("  #lngparcel3 : %d\n", 1+ ((lmr->ndivpar3 >> 0) & 0xFF));

    if (pdmdh->lmr_sz * 2 > sizeof(struct lmr_t)) {
	    uint16_t n = _2b(&lmr[1]);

	    /* Extension */
	    printf("  # Road display classes: %d\n", (n >> 10) & 0xf);
	    printf("  # Background display classes: %d\n", (n >> 5) & 0x1F);
	    printf("  # Name display classes : %d\n", n & 0x1F);
    }
  }
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

