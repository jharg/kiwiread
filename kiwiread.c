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
    printf("GetSector: %c%c%d.%d\n",
	   sa.addr & 0x80 ? 'B' : 'A',
	   sa.addr & 0x40 ? 'D' : 'S',
	   sa.addr  >> 8,
	   sa.addr & 0x3F);
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
    uint16_t		size;
    uint16_t		res1;
    uint16_t		filename;
    uint16_t		res2;

    geonum_t		upper_lat;
    geonum_t		lower_lat;
    geonum_t		left_lng;
    geonum_t		right_lng;

    uint16_t		lmr_sz;         // size in words
    uint16_t		bsmr_sz;        // size in words
    uint16_t		bmr_sz;         // size in words

    uint16_t		nlmr;
    uint16_t		nbsmr;

    /* lmr_t        lmr[var];    Level Management Records
     * bsmr_t       bsmr[var];   Block Set Management Records
     * bmt_t        bmt[var];    Block Management Tables
     */
} __attribute__((packed));

/* Level Management Record */
struct lmr_t
{
    uint16_t		header;
    uint16_t		numbers;
    uint32_t		dispflag[5];
    uint16_t		nblocksets;
    uint16_t		nblocks;
    uint16_t		nparcels;
    uint16_t		ndivpar[3];
    uint16_t		bsmr_off;      // offset in words from zdat[0]
    uint16_t		nrsize;
} __attribute__((packed));

/* Block Set Management Record */
struct bsmr_t
{
    uint16_t		header;
    uint32_t		bmt_offset;    // offset in words from zdat[0]
    uint32_t		bmt_size;      // size in words
} __attribute__((packed));

/* Reference Record of Parcel Management Information */
struct bmt_t
{
    struct sectoraddress  dsa;           // sector address
    uint16_t              size;          // size in words
    uint8_t               filename[12];  // optional
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
int xblocksets(struct lmr_t *lmr)
{
    /* Longitude */
    return 1+extract(lmr->nblocksets, 0, 7);
}

int yblocksets(struct lmr_t *lmr)
{
    /* Latitude */
    return 1+extract(lmr->nblocksets, 8, 15);
}

void showalldata()
{
    int fd, i, fdp, j, lvl, k;
    struct datavol_t dv;
    struct mhr_t *mhr;
    off_t moff, cur;
    void *zdat[34];
    char name[64];
    struct pdmdh_t *pdmdh;
    struct lmr_t *lmr;
    struct bsmr_t *bsmr;
    int nx[64] = { 0 };
    int ny[64] = { 0 };

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
	printf("  Addr:%.8" PRIx32 " Size:%.4x  Name:'%s'\n", mhr[i].dsa, mhr[i].size, name);
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
    for (i=0; i<pdmdh->nlmr; i++) {
	lmr = (struct lmr_t *)(zdat[0] + moff);
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
	swapw(&lmr->ndivpar[0]);
	swapw(&lmr->ndivpar[1]);
	swapw(&lmr->ndivpar[2]);
	swapw(&lmr->bsmr_off);
	swapw(&lmr->nrsize);

	printf("======== lmr%d:  level=%d  upper=%d lower=%d moff=%d\n", i, 
	       extract(lmr->header, 10, 15),
	       extract(lmr->header, 4, 7),
	       extract(lmr->header, 0, 3), 
	       moff);
	for (j=0; j<5; j++) {
	    printf(" disp%d: %d\n", 1+j, lmr->dispflag[j]);
	}

	lvl = extract(lmr->header, 10, 15);
	nx[lvl] = xblocksets(lmr);
	ny[lvl] = yblocksets(lmr);

	printf(" latlng block sets: %dx%d\n",
	       xblocksets(lmr), yblocksets(lmr));
	printf(" latlng blocks    : %dx%d\n",
	       1+extract(lmr->nblocks, 8, 15),
	       1+extract(lmr->nblocks, 0, 7));
	printf(" latlng parcels   : %dx%d\n",
	       1+extract(lmr->nparcels, 8, 15),
	       1+extract(lmr->nparcels, 0, 7));
	printf(" bsmroff:         : %d\n",
	       lmr->bsmr_off * 2);

	os_dump(lmr, pdmdh->lmr_sz * 2);
	moff += pdmdh->lmr_sz * 2;
    }

    /* Dump Block Set Management Records */
    for (i=0; i<pdmdh->nbsmr; i++) {
	bsmr = (struct bsmr_t *)(zdat[0] + moff);

	swapw(&bsmr->header);
	swapl(&bsmr->bmt_offset);
	swapl(&bsmr->bmt_size);

	lvl = extract(bsmr->header, 10, 15);
	printf("---- bsmr%d: level=%2d blockset=%3d moff=%d [%dx%d]\n",
	       i, extract(bsmr->header, 10, 15), extract(bsmr->header, 0, 7), moff,
	       nx[lvl], ny[lvl]);
	if (bsmr->bmt_size) {
	    struct bmt_t *bmt;
	    off_t boff, poff;
	    void *pdat;

	    printf("  bmt_offset: %d\n", bsmr->bmt_offset * 2);
	    printf("  bmt_size  : %d\n", bsmr->bmt_size * 2);
	    os_dump(zdat[0] + bsmr->bmt_offset * 2, bsmr->bmt_size * 2);

	    boff = 0;
	    while (boff < bsmr->bmt_size * 2) {
		bmt = (struct bmt_t *)(zdat[0] + bsmr->bmt_offset*2 + boff);
		swapl(&bmt->dsa);
		swapw(&bmt->size);

		printf("    Parcel addr: %lx  size:%ld\n", bmt->dsa, bmt->size);
		if (bmt->size) {
		    poff = getsector(bmt->dsa);
		    pdat = zreado(fd, bmt->size * 2, poff);
		    os_dump(pdat, bmt->size * 2);
		}
		boff += 6;
	    }
	}

	moff += sizeof(*bsmr);
    }

    /* ASSERT: count of blocksets per level == nx[lvl] * ny[lvl] */
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

