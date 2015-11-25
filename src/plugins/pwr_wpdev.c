/* 
 * Copyright 2015-2016 RNET Technologies. Under the terms of Contract
 * PO-1628426 with Sandia Corporation, there is a non-exclusive license for use of this work 
 * by or on behalf of the U.S. Government. Export of this program may require
 * a license from the United States Government.
 *
 * This file is part of the Power API Prototype software package. For license
 * information, see the LICENSE file in the top level directory of the
 * distribution.
*/

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <errno.h>

#include <pwr_dev.h>
#include <rnet_pm_api.h>

#ifdef USE_DEBUG

#define DBGX( fmt, args... ) \
{\
    fprintf( stderr, "WattProfDev:%s():%d: "fmt, __func__, __LINE__, ##args);\
}
#else

#define DBGX( fmt, args... )

#endif

#define ERRX( fmt, args... ) \
{\
    fprintf( stderr, "WattProfDev Error:%s():%d: "fmt, __func__, __LINE__, ##args);\
}


#define LOCAL_DEVICE	0
#define MAIN_TASK	0

typedef struct {
    handle_t wp_handle;
} pwr_wpdev_t;
#define PWR_WPDEV(DEV) ((pwr_wpdev_t *)(DEV))


typedef struct {
	pwr_wpdev_t *dev;
	channel_t ch;
} pwr_wpdev_fd_t;
#define PWR_WPFD(FD) ((pwr_wpdev_fd_t *)(FD))

/*openstr indicates the hw data channel with which the returning fd is associated*/
pwr_fd_t pwr_wpdev_open( plugin_devops_t* ops, const char *openstr )
{
	
	pwr_fd_t *fd;

	
	printf("Called %s with openstr %s\n",__FUNCTION__,openstr);
	int channel = atoi(openstr);
        if(channel < 0 || channel >= MAX_NUM_CHANNELS){
                errno = EINVAL;
                return NULL;
        }

	fd = calloc(1,sizeof(pwr_wpdev_fd_t));
	if(!fd){
		ERRX("Out of memory!\n");
		return NULL;
	}
	PWR_WPFD(fd)->dev = PWR_WPDEV(ops->private_data);
	PWR_WPFD(fd)->ch.channel  = channel;
	PWR_WPFD(fd)->ch.sig_type = PWR_ATTR_NOT_SPECIFIED; 
	//PWR_WPFD(fd)->ch.raw_type = PWR_ATTR_NOT_SPECIFIED; 
	//TODO: Add this function to WattProf and use it instead of the above	
	//PWR_WPFD(fd)->ch.sig_type = power_get_sigtype(channel);

   	return fd;
}

static int pwr_wpdev_close( pwr_fd_t fd )
{
    free( fd );
    return PWR_RET_SUCCESS;
}

static int pwr_wpdev_read( pwr_fd_t fd, PWR_AttrName attr, void* ptr, unsigned int len, PWR_Time* ts )
{
	int cap_size = 0,j;
	power_t *wp_capture = NULL,pow;
	pwr_wpdev_t *wp_dev = PWR_WPFD(fd)->dev;
	signal_t ch_sig = PWR_WPFD(fd)->ch.sig_type;
	cap_size = power_instant_measure(wp_dev->wp_handle,MAIN_TASK,&wp_capture,(pm_time_t *)&ts);

	printf("\n--------Sample size: %d, time: %ld--------\n",cap_size,ts);
        if(cap_size <= 0)
       		return -1;
        for(j=0;j<cap_size;j++){
        	printf("%f ",wp_capture[j]);
        }

	/*---------------------    Implemented up to HERE  ---------------------------*/
	/*FIXME - The offset calculation here is wrong, since the channel # cannot be used to calculate
        offset - some other means is required here*/	
	pow = (power_t)wp_capture[PWR_WPFD(fd)->ch.channel];
    	switch( attr ) {
        	case PWR_ATTR_VOLTAGE:
			*((power_t *)ptr) = pow;
            		break;
        	case PWR_ATTR_CURRENT:
            		*((power_t *)ptr) = pow;
            		break;
		case PWR_ATTR_AVG_POWER:
        	case PWR_ATTR_POWER:
			*((power_t *)ptr) = pow;
            		break;
        	case PWR_ATTR_MIN_POWER:
			*((power_t *)ptr) = pow;
		        break;
	        case PWR_ATTR_MAX_POWER:
			*((power_t *)ptr) = pow;
            		break;
        	case PWR_ATTR_ENERGY:
			*((power_t *)ptr) = pow;
            		break;
        	default:
            		ERRX( "Unknown or unsupported attribute (%d)\n", attr );
            		break;
    	}
    	return PWR_RET_SUCCESS;
}

static int pwr_wpdev_write( pwr_fd_t fd, PWR_AttrName type, void* ptr, unsigned int len )
{
	switch( type ) {
        	default:
            		printf( "Warning: unknown PWR writing attr (%u) requested\n", type );
            		break;
    	}	
    	return PWR_RET_SUCCESS;
}

static int pwr_wpdev_readv( pwr_fd_t fd, unsigned int arraysize, const PWR_AttrName attrs[], void* ptr,
                        PWR_Time ts[], int status[] )
{
	
	int cap_size = 0,j,i;
	power_t *wp_capture = NULL;
	pwr_wpdev_t *wp_dev = PWR_WPFD(fd)->dev;
	signal_t ch_sig = PWR_WPFD(fd)->ch.sig_type;
	cap_size = power_instant_measure(wp_dev->wp_handle,MAIN_TASK,&wp_capture,(pm_time_t *)&ts);

	printf("\n--------Sample size: %d, time: %ld--------\n",cap_size,ts);
        if(cap_size <= 0)
       		return -1;
        for(j=0;j<cap_size;j++){
        	printf("%f ",wp_capture[j]);
        }

	/*---------------------    Implemented up to HERE  ---------------------------*/
	int num_channels = cap_size / arraysize;
	for ( i = 0; i < arraysize; i++ ) {
        /*FIXME - The offset calculation here is wrong, since the channel # cannot be used to calculate
        offset - some other means is required here*/
	  power_t pow = (power_t)wp_capture[i*num_channels +  PWR_WPFD(fd)->ch.channel];
    	  switch( attrs[i] ) {
        	case PWR_ATTR_VOLTAGE:
            		break;
        	case PWR_ATTR_CURRENT:
			*((power_t *)ptr+i) = pow;
            		break;
		case PWR_ATTR_AVG_POWER:
        	case PWR_ATTR_POWER:
			*((power_t *)ptr+i) = pow;
            		break;
        	case PWR_ATTR_MIN_POWER:
			*((power_t *)ptr+i) = pow;
		        break;
	        case PWR_ATTR_MAX_POWER:
			*((power_t *)ptr+i) = pow;
            		break;
        	case PWR_ATTR_ENERGY:
			*((power_t *)ptr+i) = pow;
            		break;
        	default:
            		ERRX( "Unknown or unsupported attribute (%d)\n", attrs[i] );
            		break;
    	  }
        }
	/*Need to read the ts from the capture*/
	*ts = 0;	
    	return PWR_RET_SUCCESS;

}

static int pwr_wpdev_writev( pwr_fd_t fd, unsigned int arraysize, const PWR_AttrName attrs[], void* ptr, int status[] )
{
	int i;

    	for( i = 0; i < arraysize; i++ )
        	status[i] = pwr_wpdev_write( fd, attrs[i], (double *)ptr+i, sizeof(double) );
        return PWR_RET_SUCCESS;
}

static int pwr_wpdev_time( pwr_fd_t fd, PWR_Time *timestamp )
{
    
    return PWR_RET_SUCCESS;
}

static int pwr_wpdev_clear( pwr_fd_t fd )
{
    /*TODO soon - implement using a wattprof function*/
    return PWR_RET_SUCCESS;
}

static plugin_devops_t devOps = {
    .open   = pwr_wpdev_open,/*done*/ 
    .close  = pwr_wpdev_close,/*done*/
    .read   = pwr_wpdev_read,
    .write  = pwr_wpdev_write,
    .readv  = pwr_wpdev_readv,
    .writev = pwr_wpdev_writev,
    .time   = pwr_wpdev_time,
    .clear  = pwr_wpdev_clear,
    .stat_get = pwr_dev_stat_get,/*TODO: implement the wp version*/
    .stat_start = pwr_dev_stat_start,/*TODO: implement the wp version*/
    .stat_stop = pwr_dev_stat_stop,/*TODO: implement the wp version*/
    .stat_clear = pwr_dev_stat_clear,/*TODO: implement the wp version*/
};


static int wpdev_parse( const char *initstr, unsigned char *saddr[], unsigned int *sport )
{
    char *token;

    DBGX( "Info: received initialization string %s\n", initstr );

    
    if( (*saddr = strtok( (char *)initstr, ":" )) == 0x0 ) {
        ERRX( "Error: missing server address/port separator in initialization string %s\n", initstr );
        return -1;
    }
    if( (token = strtok( NULL, ":" )) == 0x0 ) {
        ERRX( "Error: missing server address/port separator in initialization string %s\n", initstr );
        return -1;
    }
    *sport = atoi(token);

    DBGX( "Info: extracted initialization string (SADDR=%08x, SPORT=%u)\n", *saddr, *sport );

    return PWR_RET_SUCCESS;
}

char *wpdev_get_config_filename(int dev_no){
	char *fname= malloc(100);
	sprintf(fname,"dev%d_conf.rnp",dev_no);
	return fname;
}

/*initstr refers to the WattProf board (localhost for local or IP for remote):remote port:channel
and channel is not used here*/
static plugin_devops_t* pwr_wpdev_init( const char *initstr )
{
	int numdevs;
	handle_t daqh;
	plugin_devops_t* ops;
	char saddr[64];
	int sport;
	
	DBGX( "Info: initializing PWR WattProf library\n");

	if(wpdev_parse(initstr,*saddr,sport )<0){
		return NULL;
	}
	if(strcmp(saddr,"localhost")){
		ERRX("Remote WattProf access to %s not suppported yet!\n",*saddr);
	}

        numdevs = rnet_pm_init();
        if(numdevs <=0){
                ERRX("No active WattProf devices found!\n");
		return NULL;
        }
	
	daqh = power_init(LOCAL_DEVICE,wpdev_get_config_filename(LOCAL_DEVICE));
	if(!daqh){
		ERRX("Unable to initialize WattProf device %d!\n",LOCAL_DEVICE);
                return NULL;
	}

	/*FIXME: Is this the best place for these? on in _open function calls?*/
        if(power_start_capture(daqh) < 0){
		ERRX("Unable to start data capture on Wattprof!");
		return NULL;
	}
        if(power_start_task(daqh,MAIN_TASK) < 0){
		ERRX("Unable to start measurement task on Wattprof!");
                return NULL;
	}
	ops = calloc(1,sizeof(*ops));
	
	*ops = devOps;
	ops->private_data = calloc(1,sizeof( pwr_wpdev_t ));
	PWR_WPDEV(ops->private_data)->wp_handle = daqh;	
	
	return ops;
}

static int pwr_wpdev_finalize( plugin_devops_t *ops )
{
    int err;	
    pwr_wpdev_t *wp_dev = (pwr_wpdev_t *)ops->private_data;
    if(power_stop_task(wp_dev->wp_handle,MAIN_TASK) < 0)
	ERRX("Unable to stop measurement task on Wattprof!");	
    power_stop_capture(wp_dev->wp_handle);
    err = rnet_pm_finalize();
    free(ops->private_data);
    free(ops);
    return err;
}

static plugin_dev_t dev = {
    .init   = pwr_wpdev_init, 
    .final  = pwr_wpdev_finalize,
};

plugin_dev_t* getDev() {
    return &dev;
}
