#include "mchw_rapldev.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <types.h>
#include <math.h>

#define CPU_MODEL_SANDY    42
#define CPU_MODEL_SANDY_EP 45
#define CPU_MODEL_IVY      58
#define CPU_MODEL_IVY_EP   62
#define CPU_MODEL_HASWELL  60

#define MSR_RAPL_POWER_UNIT    0x606

#define MSR_POWER_LIMIT        0x610
#define MSR_ENERGY_STATUS      0x611
#define MSR_PERF_STATUS        0x613
#define MSR_POWER_INFO         0x614

#define MSR_PP0_POWER_LIMIT    0x638
#define MSR_PP0_ENERGY_STATUS  0x639
#define MSR_PP0_POLICY_STATUS  0x63a
#define MSR_PP0_PERF_STATUS    0x63b

#define MSR_PP1_POWER_LIMIT    0x640
#define MSR_PP1_ENERGY_STATUS  0x641
#define MSR_PP1_POLICY_STATUS  0x642

#define MSR_DRAM_POWER_LIMIT   0x618
#define MSR_DRAM_ENERGY_STATUS 0x619
#define MSR_DRAM_PERF_STATUS   0x61b
#define MSR_DRAM_POWER_INFO    0x61c

#define POWER_UNITS_SHIFT    1
#define ENERGY_UNITS_SHIFT   8
#define TIME_UNITS_SHIFT    16

#define POWER_UNITS_MASK       0xf
#define ENERGY_UNITS_MASK      0x1f
#define TIME_UNITS_MASK        0xf

#define THERMAL_POWER_SHIFT  0
#define MINIMUM_POWER_SHIFT 16
#define MAXIMUM_POWER_SHIFT 32
#define WINDOW_POWER_SHIFT  48

#define THERMAL_POWER_MASK     0x7fff
#define MINIMUM_POWER_MASK     0x7fff
#define MAXIMUM_POWER_MASK     0x7fff
#define WINDOW_POWER_MASK      0x7fff

#define POWER1_LIMIT_SHIFT   0
#define WINDOW1_LIMIT_SHIFT 17
#define ENABLED1_LIMIT_BIT  15
#define CLAMPED1_LIMIT_BIT  16
#define POWER2_LIMIT_SHIFT  32
#define WINDOW2_LIMIT_SHIFT 49
#define ENABLED2_LIMIT_BIT  47
#define CLAMPED2_LIMIT_BIT  48

#define POWER1_LIMIT_MASK      0x7fff
#define WINDOW1_LIMIT_MASK     0x007f
#define POWER2_LIMIT_MASK      0x7fff
#define WINDOW2_LIMIT_MASK     0x007f

#define PP0_POLICY_SHIFT    0
#define PP1_POLICY_SHIFT    0

#define PP0_POLICY_MASK        0x001f
#define PP1_POLICY_MASK        0x001f

#define MSR(X,Y,Z) (X>>Y&Z)
#define MSR_BIT(X,Y) ((X&(1LL<<Y))?1:0)

static int rapldev_verbose = 0;

typedef enum {
    INTEL_LAYER_PKG,
    INTEL_LAYER_PP0,
    INTEL_LAYER_PP1,
    INTEL_LAYER_DRAM = INTEL_LAYER_PP1
} layer_t;

typedef struct {
    double power;  /* power units in W */
    double energy; /* energy units in J */
    double time;   /* time units in s */
} units_t;

typedef struct {
    double thermal; /* thermal specification */
    double minimum; /* minimum power */
    double maximum; /* maximum power */
    double window;  /* maximum time window */
} power_t;

typedef struct {
    double power1;  /* power limit 1 */
    double window1; /* time window 1 */
    unsigned short enabled1; /* limit 1 enabled */
    unsigned short clamped1; /* limit 1 clamped */
    double power2;  /* power limit 2 */
    double window2; /* time window 2 */
    unsigned short enabled2; /* limit 2 enabled */
    unsigned short clamped2; /* limit 2 clamped */
} limit_t;

typedef struct {
    int fd;

    int cpu_model;
    layer_t layer;

    units_t units;
    power_t power;
    limit_t limit;
} mchw_rapldev_t;
#define MCHW_RAPLDEV(X) ((mchw_rapldev_t *)(X))

typedef struct {
    mchw_rapldev_t *dev;
} mchw_raplfd_t;
#define MCHW_RAPLFD(X) ((mchw_raplfd_t *)(X))

static int rapldev_parse( const char *initstr, int *core, int *layer )
{
    char *token;

    if( rapldev_verbose )
        printf( "Info: received initialization string %s\n", initstr );

    if( (token = strtok( (char *)initstr, ":" )) == 0x0 ) {
        printf( "Error: missing core separator in initialization string %s\n", initstr );
        return -1;
    }
    *core = atoi(token);

    if( (token = strtok( NULL, ":" )) == 0x0 ) {
        printf( "Error: missing layer separator in initialization string %s\n", initstr );
        return -1;
    }
    *layer = atoi(token);
 
    if( rapldev_verbose )
        printf( "Info: extracted initialization string (CORE=%d, layer=%d)\n", *core, *layer );

    return 0;
}

static int rapldev_identify( int *cpu_model )
{
    FILE *file;
    char *str = 0x0;
    char cpuinfo[80] = "";
    int retval = 0;

    int family;
    char vendor[80] = "";

    if( (file=fopen( "/proc/cpuinfo", "r" )) == 0x0 ) {
        printf( "Error: RAPL cpuinfo open failed\n" );
        return -1;
    }

    *cpu_model = -1;
    while( (str=fgets( cpuinfo, 80, file )) != 0x0 ) {
        if( strncmp( str, "vendor_id", 8) == 0 ) {
            sscanf( str, "%*s%*s%s", vendor );
            if( strncmp( vendor, "GenuineIntel", 12 ) != 0 ) {
                printf( "Warning: %s, only Intel supported\n", vendor );
                retval = -1;
            }
        }
        else if( strncmp( str, "cpu_family", 10) == 0 ) {
            sscanf( str, "%*s%*s%*s%d", &family );
            if( family != 6 ) {
                printf( "Warning: Unsupported CPU family %d\n", family );
                retval = -1;
            }
        }
        else if( strncmp( str, "model", 5) == 0 ) {
            sscanf( str, "%*s%*s%d", cpu_model );

            switch( *cpu_model ) {
                case CPU_MODEL_SANDY:
                case CPU_MODEL_SANDY_EP:
                case CPU_MODEL_IVY:
                case CPU_MODEL_IVY_EP:
                case CPU_MODEL_HASWELL:
                    break;
                default:
                    printf( "Warning: Unsupported model %d\n", *cpu_model );
                    retval = -1;
                    break;
            }
        }
    }

    fclose( file );
    return retval;
}

static int rapldev_read( int fd, int offset, long long *msr )
{
    uint64_t value;
    int size = sizeof(uint64_t);

    if( read( fd, &value, size ) != size ) {
        printf( "Error: RAPL read failed\n" );
        return -1;
    }
 
    *msr = (long long)value;
    return 0;
}

static int rapldev_gather( int fd, int cpu_model, layer_t layer, units_t units,
                           double *energy, double *time, int *policy )
{
    long long msr;

    if( layer == INTEL_LAYER_PKG ) {
        if( rapldev_read( fd, MSR_ENERGY_STATUS, &msr ) < 0 ) {
            printf( "Error: MCHW RAPL device read failed\n" );
            return -1;
        }
        *energy = (double)msr * units.energy;

        if( cpu_model == CPU_MODEL_SANDY_EP ||
            cpu_model == CPU_MODEL_IVY_EP ) {
            if( rapldev_read( fd, MSR_PERF_STATUS, &msr ) < 0 ) {
                printf( "Error: MCHW RAPL device read failed\n" );
                return -1;
            }
            *time = (double)msr * units.time;
        }
    }
    else if( layer == INTEL_LAYER_PP0 ) {
        if( rapldev_read( fd, MSR_PP0_ENERGY_STATUS, &msr ) < 0 ) {
            printf( "Error: MCHW RAPL device read failed\n" );
            return -1;
        }
        *energy = (double)msr * units.energy;

        if( rapldev_read( fd, MSR_PP0_POLICY_STATUS, &msr ) < 0 ) {
            printf( "Error: MCHW RAPL device read failed\n" );
            return -1;
        }
        *policy = (int)(MSR(msr, PP0_POLICY_SHIFT, PP0_POLICY_MASK));

        if( cpu_model == CPU_MODEL_SANDY_EP ||
            cpu_model == CPU_MODEL_IVY_EP ) {
            if( rapldev_read( fd, MSR_PP0_PERF_STATUS, &msr ) < 0 ) {
                printf( "Error: MCHW RAPL device read failed\n" );
                return -1;
            }
            *time = (double)msr * units.time;
        }
    }
    else if( layer == INTEL_LAYER_PP1 ) {
        if( cpu_model == CPU_MODEL_SANDY  ||
            cpu_model == CPU_MODEL_IVY    ||
            cpu_model == CPU_MODEL_HASWELL ) {
            if( rapldev_read( fd, MSR_PP1_ENERGY_STATUS, &msr ) < 0 ) {
                printf( "Error: MCHW RAPL device read failed\n" );
                return -1;
            }
            *energy = (double)msr * units.energy;

            if( rapldev_read( fd, MSR_PP1_POLICY_STATUS, &msr ) < 0 ) {
                printf( "Error: MCHW RAPL device read failed\n" );
                return -1;
            }
            *policy = (int)(MSR(msr, PP1_POLICY_SHIFT, PP1_POLICY_MASK));
        }
        else {
            if( rapldev_read( fd, MSR_DRAM_ENERGY_STATUS, &msr ) < 0 ) {
                printf( "Error: MCHW RAPL device read failed\n" );
                return -1;
            }
            *energy = (double)msr * units.energy;
        }
    }

    return 0;
}

plugin_devops_t *mchw_rapldev_init( const char *initstr )
{
    char file[80] = "";
    int core = 0;
    long long msr;
    plugin_devops_t *dev = malloc( sizeof(plugin_devops_t) );
    bzero( dev, sizeof(plugin_devops_t) );

    dev->private_data = malloc( sizeof(mchw_rapldev_t) );
    bzero( dev->private_data, sizeof(mchw_rapldev_t) );

    if( rapldev_verbose ) 
        printf( "Info: MCHW RAPL device open\n" );

    if( rapldev_parse( initstr, &core, (int *)(&(MCHW_RAPLDEV(dev->private_data)->layer)) ) < 0 ) {
        printf( "Error: MCHW RAPL device initialization string %s invalid\n", initstr );
        return 0x0;
    }

    if( rapldev_identify( &(MCHW_RAPLDEV(dev->private_data)->cpu_model) ) < 0 ) {
        printf( "Error: MCHW RAPL device model identification failed\n" );
        return 0x0;
    }

    sprintf( file, "/dev/cpu/%d/msr", core );
    if( (MCHW_RAPLDEV(dev->private_data)->fd=open( file, O_RDONLY )) < 0 ) {
        printf( "Error: MCHW RAPL device open failed\n" );
        return 0x0;
    }

    if( rapldev_read( MCHW_RAPLDEV(dev->private_data)->fd, MSR_RAPL_POWER_UNIT, &msr ) < 0 ) {
        printf( "Error: MCHW RAPL device read failed\n" );
        return 0x0;
    }
    MCHW_RAPLDEV(dev->private_data)->units.power = 
        pow( 0.5, (double)(MSR(msr, POWER_UNITS_SHIFT, POWER_UNITS_MASK)) );
    MCHW_RAPLDEV(dev->private_data)->units.energy = 
        pow( 0.5, (double)(MSR(msr, ENERGY_UNITS_SHIFT, ENERGY_UNITS_MASK)) );
    MCHW_RAPLDEV(dev->private_data)->units.time =
        pow( 0.5, (double)(MSR(msr, TIME_UNITS_SHIFT, TIME_UNITS_MASK)) );

    if( rapldev_verbose ) {
        printf( "Info: units.power    - %g\n", MCHW_RAPLDEV(dev->private_data)->units.power );
        printf( "Info: units.energy   - %g\n", MCHW_RAPLDEV(dev->private_data)->units.energy );
        printf( "Info: units.time     - %g\n", MCHW_RAPLDEV(dev->private_data)->units.time );
    }

    if( rapldev_read( MCHW_RAPLDEV(dev->private_data)->fd, MSR_POWER_INFO, &msr ) < 0 ) {
        printf( "Error: MCHW RAPL device read failed\n" );
        return 0x0;
    }
    MCHW_RAPLDEV(dev->private_data)->power.thermal =
        (double)(MSR(msr, THERMAL_POWER_SHIFT, THERMAL_POWER_MASK));
    MCHW_RAPLDEV(dev->private_data)->power.minimum =
        (double)(MSR(msr, MINIMUM_POWER_SHIFT, MINIMUM_POWER_MASK));
    MCHW_RAPLDEV(dev->private_data)->power.maximum =
        (double)(MSR(msr, MAXIMUM_POWER_SHIFT, MAXIMUM_POWER_MASK));
    MCHW_RAPLDEV(dev->private_data)->power.window =
        (double)(MSR(msr, WINDOW_POWER_SHIFT, WINDOW_POWER_MASK));

    if( rapldev_verbose ) {
        printf( "Info: power.thermal  - %g\n", MCHW_RAPLDEV(dev->private_data)->power.thermal );
        printf( "Info: power.minimum  - %g\n", MCHW_RAPLDEV(dev->private_data)->power.minimum );
        printf( "Info: power.maximum  - %g\n", MCHW_RAPLDEV(dev->private_data)->power.maximum );
        printf( "Info: power.window   - %g\n", MCHW_RAPLDEV(dev->private_data)->power.window );
    }

    if( rapldev_read( MCHW_RAPLDEV(dev->private_data)->fd, MSR_POWER_LIMIT, &msr ) < 0 ) {
        printf( "Error: MCHW RAPL device read failed\n" );
        return 0x0;
    }
    MCHW_RAPLDEV(dev->private_data)->limit.power1 =
        (double)(MSR(msr, POWER1_LIMIT_SHIFT, POWER1_LIMIT_MASK));
    MCHW_RAPLDEV(dev->private_data)->limit.window1 =
        (double)(MSR(msr, WINDOW1_LIMIT_SHIFT, WINDOW1_LIMIT_MASK));
    MCHW_RAPLDEV(dev->private_data)->limit.enabled1 =
        (unsigned short)(MSR_BIT(msr, ENABLED1_LIMIT_BIT));
    MCHW_RAPLDEV(dev->private_data)->limit.clamped1 =
        (unsigned short)(MSR_BIT(msr, CLAMPED1_LIMIT_BIT));
    MCHW_RAPLDEV(dev->private_data)->limit.power2 =
        (double)(MSR(msr, POWER2_LIMIT_SHIFT, POWER2_LIMIT_MASK));
    MCHW_RAPLDEV(dev->private_data)->limit.window2 =
        (double)(MSR(msr, WINDOW2_LIMIT_SHIFT, WINDOW2_LIMIT_MASK));
    MCHW_RAPLDEV(dev->private_data)->limit.enabled2 =
        (unsigned short)(MSR_BIT(msr, ENABLED2_LIMIT_BIT));
    MCHW_RAPLDEV(dev->private_data)->limit.clamped2 =
        (unsigned short)(MSR_BIT(msr, CLAMPED2_LIMIT_BIT));
 
    if( rapldev_verbose ) {
        printf( "Info: limit.power1   - %g\n", MCHW_RAPLDEV(dev->private_data)->limit.power1 );
        printf( "Info: limit.window1  - %g\n", MCHW_RAPLDEV(dev->private_data)->limit.window1 );
        printf( "Info: limit.enabled1 - %u\n", MCHW_RAPLDEV(dev->private_data)->limit.enabled1 );
        printf( "Info: limit.clamped1 - %u\n", MCHW_RAPLDEV(dev->private_data)->limit.clamped1 );
        printf( "Info: limit.power2   - %g\n", MCHW_RAPLDEV(dev->private_data)->limit.power1 );
        printf( "Info: limit.window2  - %g\n", MCHW_RAPLDEV(dev->private_data)->limit.window1 );
        printf( "Info: limit.enabled2 - %u\n", MCHW_RAPLDEV(dev->private_data)->limit.enabled2 );
        printf( "Info: limit.clamped2 - %u\n", MCHW_RAPLDEV(dev->private_data)->limit.clamped2 );
    }

    return dev;
}

int mchw_rapldev_final( plugin_devops_t *dev )
{
    if( rapldev_verbose ) 
        printf( "Info: MCHW RAPL device close\n" );

    close( MCHW_RAPLDEV(dev->private_data)->fd );
    free( dev->private_data );
    free( dev );

    return 0;
}

pwr_fd_t mchw_rapldev_open( plugin_devops_t *dev, const char *openstr )
{
    pwr_fd_t *fd = malloc( sizeof(mchw_raplfd_t) );
    MCHW_RAPLFD(fd)->dev = MCHW_RAPLDEV(dev->private_data);

    return 0;
}

int mchw_rapldev_close( pwr_fd_t fd )
{
    MCHW_RAPLFD(fd)->dev = 0x0;
    free( fd );

    return 0;
}

int mchw_rapldev_read( pwr_fd_t fd, PWR_AttrName attr, void *value, unsigned int len, PWR_Time *timestamp )
{
    double energy = 0.0;
    double time = 0.0;
    int policy = 0;

    if( rapldev_verbose ) 
        printf( "Info: MCHW RAPL device read\n" );

    if( len != sizeof(double) ) {
        printf( "Error: value field size of %u incorrect, should be %ld\n", len, sizeof(double) );
        return -1;
    }

    if( rapldev_gather( MCHW_RAPLDEV(MCHW_RAPLFD(fd)->dev)->fd,
                        MCHW_RAPLDEV(MCHW_RAPLFD(fd)->dev)->cpu_model,
                        MCHW_RAPLDEV(MCHW_RAPLFD(fd)->dev)->layer,
                        MCHW_RAPLDEV(MCHW_RAPLFD(fd)->dev)->units,
                        &energy, &time, &policy ) < 0 ) {
        printf( "Error: MCHW RAPL device gather failed\n" );
        return -1;
    }

    switch( attr ) {
        case PWR_ATTR_ENERGY:
            *((double *)value) = energy;
            break;
        default:
            printf( "Warning: unknown MCHW reading attr requested\n" );
            break;
    }
    *timestamp = (unsigned int)time*1000000000ULL + 
                 (time-(unsigned int)time)*1000000000ULL;

    return 0;
}

int mchw_rapldev_write( pwr_fd_t fd, PWR_AttrName attr, void *value, unsigned int len )
{
    return 0;
}

int mchw_rapldev_readv( pwr_fd_t fd, unsigned int arraysize,
    const PWR_AttrName attrs[], void *values, PWR_Time timestamp[], int status[] )
{
    unsigned int i;
    double time = 0.0;
    double energy = 0.0;
    int policy = 0;

    if( rapldev_verbose ) 
        printf( "Info: MCHW RAPL device read\n" );

    if( rapldev_gather( MCHW_RAPLDEV(MCHW_RAPLFD(fd)->dev)->fd,
                        MCHW_RAPLDEV(MCHW_RAPLFD(fd)->dev)->cpu_model,
                        MCHW_RAPLDEV(MCHW_RAPLFD(fd)->dev)->layer,
                        MCHW_RAPLDEV(MCHW_RAPLFD(fd)->dev)->units,
                        &energy, &time, &policy ) < 0 ) {
        printf( "Error: MCHW RAPL device gather failed\n" );
        return -1;
    }

    for( i = 0; i < arraysize; i++ ) {
        switch( attrs[i] ) {
            case PWR_ATTR_ENERGY:
                *((double *)values+i) = energy;
                break;
            default:
                printf( "Warning: unknown MCHW reading attr requested\n" );
                return 1;
        }
        *timestamp = (unsigned int)time*1000000000ULL + 
                     (time-(unsigned int)time)*1000000000ULL;
    }

    return 0;
}

int mchw_rapldev_writev( pwr_fd_t fd, unsigned int arraysize,
    const PWR_AttrName attrs[], void *values, int status[] )
{
    if( rapldev_verbose ) 
        printf( "Info: MCHW RAPL device write\n" );

    return 0;
}

int mchw_rapldev_time( pwr_fd_t fd, PWR_Time *timestamp )
{
    double value;

    if( rapldev_verbose ) 
        printf( "Info: MCHW RAPL device time\n" );

    return mchw_rapldev_read( fd, PWR_ATTR_POWER, &value, sizeof(double), timestamp );
}

int mchw_rapldev_clear( pwr_fd_t fd )
{
    return 0;
}

static plugin_devops_t devops = {
    .open   = mchw_rapldev_open,
    .close  = mchw_rapldev_close,
    .read   = mchw_rapldev_read,
    .write  = mchw_rapldev_write,
    .readv  = mchw_rapldev_readv,
    .writev = mchw_rapldev_writev,
    .time   = mchw_rapldev_time,
    .clear  = mchw_rapldev_clear
};

static plugin_dev_t dev = {
    .init   = mchw_rapldev_init,
    .final  = mchw_rapldev_final,
};

plugin_dev_t* getDev() {
    return &dev;
}