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


#define DEFAULT_DEVICE	0
#define MAIN_TASK	0

typedef struct {
    handle_t wp_handle;
} pwr_wpdev_t;


typedef struct {
	pwr_wpdev_t *dev;
	component_t component;
} pwr_wpdev_fd_t;

/*openstr indicates the hw data channel with which the returning fd is associated*/
static pwr_wpdev_fd_t pwr_wpdev_open( plugin_devops_t* ops, const char *openstr )
{
	
	pwr_wpdev_fd_t *fd;

	channel_t channel = (channel_t)atoi(openstr);
        if(channel < 0 || channel >= MAX_NUM_CHANNELS){
                errno = EINVAL;
                return 0;
        }

	fd = calloc(1,sizeof(pwr_wpdev_fd_t));
	if(!fd){
		ERRX("Out of memory!\n");
		return 0;
	}
	fd->dev	 = (pwr_wpdev_t *)(dev->private_data);
	fd->ch = channel;

   	return fd;
}

static int pwr_wpdev_close( pwr_wpdev_fd_t fd )
{
    free( fd );
    return 0;
}

static int pwr_wpdev_read( pwr_wpdev_fd_t fd, PWR_AttrName attr, void* ptr, unsigned int len, PWR_Time* ts )
{
	int cap_size = 0;
	power_t *wp_capture = NULL;
	pwr_wpdev_t *wp_dev = fd->dev;
	channel_attr_t ch_attr = power_get_channel_attributes(fd->ch);
	tag_hanle_t tag = power_instant_measure(wp_dev->wp_handle,MAIN_TASK,&wp_capture,&cap_size);
	if(tag == INVALID_TAG)
		return -1;
	
	/*FIXME soon: At this point returning invalid values!*/
    	switch( attr ) {
        	case PWR_ATTR_VOLTAGE:
            		break;
        	case PWR_ATTR_CURRENT:
            		*((power_t *)ptr) = (power_t)INVALID_POWER;
            		break;
		case PWR_ATTR_AVG_POWER:
        	case PWR_ATTR_POWER:
			*((power_t *)ptr) = (power_t)INVALID_POWER;
            		break;
        	case PWR_ATTR_MIN_POWER:
			*((power_t *)ptr) = (power_t)INVALID_POWER;
		        break;
	        case PWR_ATTR_MAX_POWER:
			*((power_t *)ptr) = (power_t)INVALID_POWER;
            		break;
        	case PWR_ATTR_ENERGY:
			*((power_t *)ptr) = (power_t)INVALID_POWER;
            		break;
        	default:
            		ERRX( "Unknown or unsupported attribute (%d)\n", attr );
            		break;
    	}
	/*Need to read the ts from the capture*/
	*ts = 0;	
    	return 0;
}

static int dummy_dev_write( pwr_fd_t fd, PWR_AttrName type, void* ptr, unsigned int len )
{
    DBGX("type=%s %f\n",attrNameToString(type), *(double*)ptr);

    ((dummyFdInfo_t*) fd)->buffers[type].values[0] = *(double*)ptr;
    return PWR_RET_SUCCESS;
}

static int dummy_dev_readv( pwr_fd_t fd, unsigned int arraysize, const PWR_AttrName attrs[], void* buf,
                        PWR_Time ts[], int status[] )
{
    int i;
    for ( i = 0; i < arraysize; i++ ) {

        ((double*)buf)[i] = ((dummyFdInfo_t*) fd)->buffers[attrs[i]].values[0];

        DBGX("type=%s %f\n",attrNameToString(attrs[i]), ((double*)buf)[i]);

        ts[i] = getTime();

        status[i] = PWR_RET_SUCCESS;
    }
    return PWR_RET_SUCCESS;
}

static int dummy_dev_writev( pwr_fd_t fd, unsigned int arraysize, const PWR_AttrName attrs[], void* buf, int status[] )
{
    int i;
    DBGX("num attributes %d\n",arraysize);
    for ( i = 0; i < arraysize; i++ ) {
        DBGX("type=%s %f\n",attrNameToString(attrs[i]), ((double*)buf)[i]);

        ((dummyFdInfo_t*) fd)->buffers[attrs[i]].values[0] = ((double*)buf)[i];

        status[i] = PWR_RET_SUCCESS;
    }
    return PWR_RET_SUCCESS;
}

static int dummy_dev_time( pwr_fd_t fd, PWR_Time *timestamp )
{
    DBGX("\n");

    return 0;
}

static int dummy_dev_clear( pwr_fd_t fd )
{
    DBGX("\n");

    return 0;
}

static int dummy_dev_stat_start( pwr_fd_t fd, PWR_AttrName name )
{
    buffer_t* ptr = &((dummyFdInfo_t*) fd)->buffers[name];
    DBGX("\n");
	ptr->timeStamps[0] = getTime();
	return PWR_RET_SUCCESS;
}

static int dummy_dev_stat_stop( pwr_fd_t fd, PWR_AttrName name )
{
	return PWR_RET_SUCCESS;
}

static int dummy_dev_stat_clear( pwr_fd_t fd, PWR_AttrName name )
{
	return PWR_RET_SUCCESS;
}

static int dummy_dev_stat_get( pwr_fd_t fd, PWR_AttrName name, statOp_t op,
                                    void* result, PWR_StatTimes* ts )
{
    buffer_t* ptr = &((dummyFdInfo_t*) fd)->buffers[name];
    DBGX("\n");
	ts->start = ptr->timeStamps[0]; 
	ts->stop = getTime();
	return  op(BUFFER_LEN, ptr->values,result,&ts->instant);
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

/*initstr refers to the WattProf configuration file (.rnp)*/
static plugin_devops_t* pwr_wpdev_init( const char *initstr )
{
	int numdevs;
	handle_t daqh;
	plugin_devops_t* ops;
	
	DBGX( "Info: initializing PWR WattProf library\n");

        numdevs = rnet_pm_init();
        if(numdevs <=0){
                ERRX("No active WattProf devices found!\n");
		return NULL;
        }
	
	daqh = power_init(DEFAULT_DEVICE,initstr);
	if(!daqh){
		ERRX("Unable to initialize WattProf device %d!\n",DEFAULT_DEVICE);
                return NULL;
	}

	/*FIXME: Is this the best place for these? on in _open function calls?*/
        if(power_start_capture(daqh) < 0){
		ERRX("Unable to start data capture on Wattprof!");
		return NULL;
	}
        if(power_start_task(daqh,MAIN_TASK) < 0){
		ERRX("Unable to start measurement
task on Wattprof!");
                return NULL;
	}
	ops = calloc(1,sizeof(*ops));
	
	*ops = devOps;
	ops->private_data = calloc(1,sizeof( pwr_wpdev_t ));
	(pwr_wpdev_t *)(ops->private_data)->wp_handle = daqh;	
	
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
