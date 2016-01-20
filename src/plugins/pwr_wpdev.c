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
#include <stdbool.h>
#include <string.h>
#include <sys/time.h>
#include <errno.h>
#include <time.h>

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
#define REMOTE_DEVICE	0
#define MAIN_TASK	0

//Make it zero if you do not want to go to card for the very last sample and want 
//the latest sample already pushed up by the card (might be from an earlier time)
#define READ_FROM_CARD	0

typedef struct {
    handle_t wp_handle;
    rnet_pm_lib *rnet_pm_api;
    bool wp_device_local;
} pwr_wpdev_t;
#define PWR_WPDEV(DEV) ((pwr_wpdev_t *)(DEV))


typedef struct {
	pwr_wpdev_t *dev;
	component_t comp;
} pwr_wpdev_fd_t;
#define PWR_WPFD(FD) ((pwr_wpdev_fd_t *)(FD))

static rnet_pm_lib local_lib = {
	.pm_init = rnet_pm_init,
	.pm_finalize = rnet_pm_finalize,
	.power_init = power_init,
	.power_finalize = power_finalize,
	.power_add_task = power_add_task,
	.power_start_task = power_start_task,
	.power_stop_task = power_stop_task,
	.power_get_task_opcode = power_get_task_opcode,
	.power_set_task_callback = power_set_task_callback,
	.power_get_task_capture_file = power_get_task_capture_file,
	.power_get_tag_file = power_get_tag_file,
	.power_remove_task = power_remove_task,
	.power_start_capture = power_start_capture,
	.power_stop_capture = power_stop_capture,
	.power_start_measure = power_start_measure,
	.power_end_measure = power_end_measure,
	.power_get_measurement_period = power_get_measurement_period,
	.power_single_measure = power_single_measure,
	.power_instant_measure = power_instant_measure,
	.power_instant_measure_comp = power_instant_measure_comp,
	.power_next_measure = power_next_measure,
	.power_next_measure_comp = power_next_measure_comp,
	.power_get_channel_list = power_get_channel_list,
	.power_get_components = power_get_components,
	.power_get_component = power_get_component,
	.power_get_num_tasks = power_get_num_tasks,
	.power_get_channel_data = power_get_channel_data
};

static rnet_pm_lib remote_lib = {
        .pm_init = rnet_pm_init_remote,
        .pm_finalize = rnet_pm_finalize_remote,
        .power_init = power_init_remote,
        .power_finalize = power_finalize_remote,
        .power_add_task = power_add_task_remote,
        .power_start_task = power_start_task_remote,
        .power_stop_task = power_stop_task_remote,
        .power_get_task_opcode = power_get_task_opcode_remote,
        .power_set_task_callback = power_set_task_callback_remote,
        .power_get_task_capture_file = power_get_task_capture_file_remote,
        .power_get_tag_file = power_get_tag_file_remote,
        .power_remove_task = power_remove_task_remote,
        .power_start_capture = power_start_capture_remote,
        .power_stop_capture = power_stop_capture_remote,
        .power_start_measure = power_start_measure_remote,
        .power_end_measure = power_end_measure_remote,
        .power_get_measurement_period = power_get_measurement_period_remote,
        .power_single_measure = power_single_measure_remote,
        .power_instant_measure = power_instant_measure_remote,
        .power_instant_measure_comp = power_instant_measure_comp_remote,
        .power_next_measure = power_next_measure_remote,
	.power_next_measure_comp = power_next_measure_comp_remote,
        .power_get_channel_list = power_get_channel_list_remote,
        .power_get_components = power_get_components_remote,
        .power_get_component = power_get_component_remote,
        .power_get_num_tasks = power_get_num_tasks_remote,
        .power_get_channel_data = power_get_channel_data_remote
};


/*openstr indicates the component with which the returning fd is associated*/
pwr_fd_t pwr_wpdev_open( plugin_devops_t* ops, const char *openstr )
{
	
	pwr_fd_t *fd;

	
	int comp_id = atoi(openstr);
        if(comp_id < 0 || comp_id >= RNET_MAX_COMPONENTS){
                errno = EINVAL;
                return NULL;
        }
	fd = calloc(1,sizeof(pwr_wpdev_fd_t));
	if(!fd){
		ERRX("Out of memory!\n");
		return NULL;
	}
	PWR_WPFD(fd)->dev = PWR_WPDEV(ops->private_data);
	PWR_WPFD(fd)->comp.comp_id = comp_id;
	if(!PWR_WPFD(fd)->dev->rnet_pm_api->power_get_component(PWR_WPFD(fd)->dev->wp_handle,comp_id,&PWR_WPFD(fd)->comp)){
		ERRX("Error in getting the component with id %d\n",comp_id);
		return NULL;
	}
   	return fd;
}

static int pwr_wpdev_close( pwr_fd_t fd )
{
    free( fd );
    return PWR_RET_SUCCESS;
}

static int pwr_wpdev_read( pwr_fd_t fd, PWR_AttrName attr, void* ptr, unsigned int len, PWR_Time* ts )
{
	int cap_size = 0,j,num_channels;
	power_t *wp_capture = NULL,pow;
	pwr_wpdev_t *wp_dev = PWR_WPFD(fd)->dev;
	int channel_list[MAX_NUM_CHANNELS];
	pm_time_t time;

	if( len != sizeof(double) ) {
        	ERRX( "Error: value field size of %u incorrect, should be %ld\n",len,sizeof(power_t));
        	return -1;
    	}

	DBGX("Info: Reading from WattProf channel: %d\n",PWR_WPFD(fd)->comp.comp_id);


	cap_size = wp_dev->rnet_pm_api->power_instant_measure_comp(wp_dev->wp_handle,MAIN_TASK,
			PWR_WPFD(fd)->comp.comp_id,&wp_capture,(pm_time_t *)&time,READ_FROM_CARD);
	

	DBGX("\n--------Sample size: %d, time: %ld--------\n",cap_size,time);
	for(j=0;j<cap_size;j++)	
		printf("%f ",*(wp_capture + j));
	printf(" Time: %ld \n",time);

	if(cap_size <= 0){
		ERRX("No data returned!");
                return -1;
	}
	*ts = (PWR_Time)time;
    	switch( attr ) {
        	case PWR_ATTR_VOLTAGE:
			*((double *)ptr) = (double)(power_t)wp_capture[SIG_VOLTAGE];
			break; 
        	case PWR_ATTR_CURRENT:
			*((double *)ptr) = (double)(power_t)wp_capture[SIG_CURRENT];
                        break;
		case PWR_ATTR_MIN_POWER:
                case PWR_ATTR_MAX_POWER:
		case PWR_ATTR_AVG_POWER:
			*((double *)ptr) = 0;//invalid
			break;
        	case PWR_ATTR_POWER:
			*((double *)ptr) = (double)(power_t)wp_capture[SIG_POWER];
                        break;
        	case PWR_ATTR_ENERGY:
			*((double *)ptr) = (double)(power_t)wp_capture[SIG_ENERGY];
            		break;
        	default:
            		ERRX( "Unknown or unsupported attribute (%d)\n", attr );
            		break;
    	}//switch
    	return PWR_RET_SUCCESS;
}

static int pwr_wpdev_write( pwr_fd_t fd, PWR_AttrName type, void* ptr, unsigned int len )
{
	switch( type ) {
        	default:
            		ERRX( "Warning: unknown PWR writing attr (%u) requested\n", type );
            		break;
    	}	
    	return PWR_RET_SUCCESS;
}


static int pwr_wpdev_readv( pwr_fd_t fd, unsigned int arraysize, const PWR_AttrName attrs[], void* ptr,
                        PWR_Time ts[], int status[] )
{
	int cap_size = 0,i,num_channels;
	power_t *wp_capture = NULL,pow;
	pwr_wpdev_t *wp_dev = PWR_WPFD(fd)->dev;
	int channel_list[MAX_NUM_CHANNELS];
	pm_time_t time;

	DBGX("Info: Readving from WattProf component: %d\n",PWR_WPFD(fd)->comp.comp_id);


	cap_size = wp_dev->rnet_pm_api->power_instant_measure_comp(wp_dev->wp_handle,MAIN_TASK,
			PWR_WPFD(fd)->comp.comp_id,&wp_capture,(pm_time_t *)&time,READ_FROM_CARD);

	DBGX("\n--------Sample size: %d, time: %ld--------\n",cap_size,time);

	if(cap_size <= 0){
		ERRX("No data returned!");
                return -1;
	}

	for ( i = 0; i < arraysize; i++ ) {
	   ts[i] = (PWR_Time)time;
    	   switch( attrs[i] ) {
        	case PWR_ATTR_VOLTAGE:
			((double *)ptr)[i] = (double)(power_t)wp_capture[SIG_VOLTAGE];
			status[i] = 0;/*FIXME: always correct?*/
			break; 
        	case PWR_ATTR_CURRENT:
			((double *)ptr)[i] = (double)(power_t)wp_capture[SIG_CURRENT];
			status[i] = 0;/*FIXME: always correct?*/
                        break;
		case PWR_ATTR_MIN_POWER:
                case PWR_ATTR_MAX_POWER:
		case PWR_ATTR_AVG_POWER:
			((double *)ptr)[i] = 0;//invalid
			status[i] = -1;
			break;
        	case PWR_ATTR_POWER:
			((double *)ptr)[i] = (double)(power_t)wp_capture[SIG_POWER];
			status[i] = 0;/*FIXME: always correct?*/
                        break;
        	case PWR_ATTR_ENERGY:
			((double *)ptr)[i] = (double)(power_t)wp_capture[SIG_ENERGY];
			status[i] = 0;/*FIXME: always correct?*/
            		break;
        	default:
            		ERRX( "Unknown or unsupported attribute (%d)\n", attrs[i] );
            		break;
    	   }//switch
	}
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
    double value;
    /*FIXME: For now we use the read function to return time*/
    return pwr_wpdev_read(fd, PWR_ATTR_POWER,&value ,sizeof(value), timestamp); 
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


static int wpdev_parse( const char *initstr, char *saddr[], unsigned int *sport )
{
    char *token;

    DBGX( "Info: received initialization string %s\n", initstr );

    
    if( (*saddr = strtok( (char *)initstr, ":" )) == 0x0 ) {
        ERRX( "Error: missing server address/port separator in initialization string %s\n", initstr );
        return -1;
    }
    DBGX( "Info: retreived saddr\n");
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
	char *saddr;
	int sport,daq_no;
	rnet_pm_lib *rnet_pm_api;
	bool wp_device_local;
	struct sockaddr local_agent;
	
	DBGX( "Info: initializing PWR WattProf library\n");

	if(wpdev_parse(initstr,&saddr,&sport )<0){
		return NULL;
	}
	if(strcmp(saddr,"localhost")){
		DBGX( "Info: Connecting to remote library...");
		rnet_pm_api = &remote_lib;
		wp_device_local = false;
		daq_no = REMOTE_DEVICE;
	}else{
		DBGX( "Info: Connecting to local library...");
		rnet_pm_api = &local_lib;
		wp_device_local = true;
		daq_no = LOCAL_DEVICE;
	}
        numdevs = rnet_pm_api->pm_init();
        if(numdevs <=0){
                ERRX("No active WattProf devices found!\n");
		return NULL;
        }
	DBGX( "Info: init wp dev with config file %s, saddr %s, sport %d\n",wpdev_get_config_filename(daq_no),saddr,sport);	
	if(wp_device_local)
		daqh = rnet_pm_api->power_init(daq_no,wpdev_get_config_filename(daq_no));
	else
		daqh = rnet_pm_api->power_init(saddr,sport,wpdev_get_config_filename(daq_no),&local_agent);
	if(!daqh){
		ERRX("Unable to initialize WattProf device %d!\n",daq_no);
                return NULL;
	}

	/*FIXME: Is this the best place for these? on in _open function calls?*/
        if(rnet_pm_api->power_start_capture(daqh) < 0){
		ERRX("Unable to start data capture on Wattprof!");
		return NULL;
	}
        if(rnet_pm_api->power_start_task(daqh,MAIN_TASK) < 0){
		ERRX("Unable to start measurement task on Wattprof!");
                return NULL;
	}
	ops = calloc(1,sizeof(*ops));
	
	*ops = devOps;
	ops->private_data = calloc(1,sizeof( pwr_wpdev_t ));
	PWR_WPDEV(ops->private_data)->wp_handle = daqh;
	PWR_WPDEV(ops->private_data)->rnet_pm_api = rnet_pm_api;
	PWR_WPDEV(ops->private_data)->wp_device_local = wp_device_local;
	
	return ops;
}

static int pwr_wpdev_finalize( plugin_devops_t *ops )
{
    int err;	
    pwr_wpdev_t *wp_dev = (pwr_wpdev_t *)ops->private_data;
    if((err = wp_dev->rnet_pm_api->power_stop_task(wp_dev->wp_handle,MAIN_TASK)) < 0){
	ERRX("Unable to stop measurement task on Wattprof!");	
	goto return_label;
    }
    wp_dev->rnet_pm_api->power_stop_capture(wp_dev->wp_handle);
    if(err = wp_dev->rnet_pm_api->power_finalize(wp_dev->wp_handle)){
	ERRX("Unable to finalize Wattprof device!");
	goto return_label;
    }
    err = wp_dev->rnet_pm_api->pm_finalize();
return_label:
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
