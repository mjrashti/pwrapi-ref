/* 
 * Copyright 2014-2015 Sandia Corporation. Under the terms of Contract
 * DE-AC04-94AL85000, there is a non-exclusive license for use of this work 
 * by or on behalf of the U.S. Government. Export of this program may require
 * a license from the United States Government.
 *
 * This file is part of the Power API Prototype software package. For license
 * information, see the LICENSE file in the top level directory of the
 * distribution.
*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>

#include <pwr.h> 

char* myctime(const time_t *timep);

int main( int argc, char* argv[] )
{
    PWR_Grp     grp;
    PWR_Obj     self;
    PWR_Cntxt   cntxt;
    time_t      time;
    int         retval;
    double       value;
    PWR_Time ts;
	PWR_Status  status;

    // Get a context
    cntxt = PWR_CntxtInit( PWR_CNTXT_DEFAULT, PWR_ROLE_APP, "App" );
    assert( cntxt );
    self = PWR_CntxtGetEntryPoint( cntxt );
    assert( self );
    
    printf("I'm a `%s`\n", PWR_ObjGetTypeString( PWR_ObjGetType( self ) ) ); 

    PWR_Obj parent = PWR_ObjGetParent( self );
    assert( ! parent );

    PWR_Grp children = PWR_ObjGetChildren( self );
    assert( children );

    int i;
    for ( i = 0; i < PWR_GrpGetNumObjs(children); i++ ) {
        printf("child %s\n", PWR_ObjGetName( 
                        PWR_GrpGetObjByIndx( children, i ) ) );
    }

    retval = PWR_ObjAttrGetValue( self, PWR_ATTR_VOLTAGE, &value, &ts );
    assert( retval == PWR_RET_INVALID );

    retval = PWR_ObjAttrGetValue( self, PWR_ATTR_POWER, &value, &ts );
    assert( retval == PWR_RET_SUCCESS );

    PWR_TimeConvert( ts, &time );
    printf("PWR_ObjAttrGetValue(PWR_ATTR_POWER) value=%f ts=`%s`\n",
													value,myctime(&time));
    
    value = 25.812;
    printf("PWR_ObjAttrSetValue(PWR_ATTR_POWER) value=%f\n",value);
    retval = PWR_ObjAttrSetValue( self, PWR_ATTR_POWER, &value );
    assert( retval == PWR_RET_SUCCESS );

    
    PWR_AttrName name = PWR_ATTR_POWER;
     
    status = PWR_StatusCreate();

    retval = PWR_ObjAttrGetValues( self, 1, &name, &value, &ts, status );  
    assert( retval == PWR_RET_SUCCESS );

    PWR_TimeConvert( ts, &time );
    printf("PWR_ObjAttrGetValues(PWR_ATTR_POWER) value=%f ts=`%s`\n", 
													value, myctime( &time ) );

    value = 100.10;
    printf("PWR_ObjAttrSetValues(PWR_ATTR_POWER) value=%f\n",value);
    retval = PWR_ObjAttrSetValues( self, 1, &name, &value, status );  
    assert( retval == PWR_RET_SUCCESS );

    retval = PWR_ObjAttrGetValue( self, PWR_ATTR_POWER, &value, &ts );
    assert( retval == PWR_RET_SUCCESS );

    assert( value == 200.20 );

    PWR_TimeConvert( ts, &time );
    printf("PWR_ObjAttrGetValue(PWR_ATTR_POWER) value=%f ts=`%s`\n",
													value,myctime(&time));

    grp = PWR_CntxtGetGrpByType( cntxt, PWR_OBJ_CORE );
    assert( grp );
	assert( PWR_GrpGetNumObjs( grp ) );

    value = 0.1;
    printf("PWR_GrpAttrSetValue(PWR_ATTR_POWER) value=%f\n", value);
    retval = PWR_GrpAttrSetValue( grp, PWR_ATTR_POWER, &value, status );
    assert( retval == PWR_RET_SUCCESS );

    retval = PWR_ObjAttrGetValue( self, PWR_ATTR_POWER, &value, &ts );
    assert( retval == PWR_RET_SUCCESS );

    assert( value == 0.2 );

    PWR_TimeConvert( ts, &time );
    printf("PWR_ObjAttrGetValue(PWR_ATTR_POWER) value=%f ts=`%s`\n",
													value,myctime(&time));

	PWR_Obj* core = PWR_GrpGetObjByIndx( grp, 0 );
	assert( core );

	PWR_Stat coreStat = PWR_ObjCreateStat( core, PWR_ATTR_POWER,
				PWR_ATTR_STAT_AVG );
	assert( coreStat );

	retval = PWR_StatStart( coreStat );
    assert( retval == PWR_RET_SUCCESS );

	usleep(10);
	PWR_StatTimes statTimes;
	retval = PWR_StatGetValue( coreStat, &value, &statTimes);
    assert( retval == PWR_RET_SUCCESS );


    printf("PWR_StatGetValue(PWR_ATTR_POWER) value=%f\n", value );
    printf("PWR_StatGetValue(PWR_ATTR_POWER) start=%llu\n", 
                                    (long long)statTimes.start );

   	printf("PWR_StatGetValue(PWR_ATTR_POWER) stop=%llu\n",
                                    (long long) statTimes.stop );

	if ( statTimes.instant != PWR_TIME_NOT_SET ) {
    	printf("PWR_StatGetValue(PWR_ATTR_POWER) instant=%llu\n",
									(long long )statTimes.instant );
	}

	PWR_StatDestroy( coreStat );

	PWR_CntxtDestroy( cntxt );
    return 0;
}
char* myctime(const time_t *timep)
{
    char* tmp = ctime(timep);
    tmp[ strlen(tmp) - 1 ] = 0; 
    return tmp;
}
