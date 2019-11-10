/********************************************************************
* Description: xyzac-switchkins.c
*   Derived from a work by Fred Proctor & Will Shackleford
*   and patch on forum:
*   https://forum.linuxcnc.org/10-advanced-configuration/31813-tcp-5-axis-kinematics?start=170#149538
*   based on work of
*   Rushabh Loladia: https://github.com/rushabhGH?tab=repositories (switchKins branch)
*
*  switchkins_type: 0 ==> trivkins, 1 ==> xyzac
*
* License: GPL Version 2
* Copyright (c) 2009 All rights reserved.
*
********************************************************************/

#include "motion.h"
#include "hal.h"
#include "rtapi.h"
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "rtapi_math.h"
#include "rtapi_string.h"
#include "kinematics.h"

// sequential joint number assignments
#define JX 0
#define JY 1
#define JZ 2

#define JA 3
#define JC 4

struct haldata {
    hal_float_t *x_rot_point;
    hal_float_t *y_rot_point;
    hal_float_t *z_rot_point;
    hal_float_t *z_offset;
    hal_float_t *y_offset;
    hal_float_t *tool_offset;
} *haldata;

struct data {
    hal_s32_t joints[EMCMOT_MAX_JOINTS];
    hal_u32_t switchkins_type;
} *data;

#define SET(f) pos->f = (joints[i])

int kinematicsSwitchable() {return 1;}

int kinematicsSwitch(int switchkins_type)
{
    data->switchkins_type = switchkins_type;
    switch (data->switchkins_type) {
       case 1: rtapi_print_msg(RTAPI_MSG_INFO,"xyzac-switchkins: kinematicsSwitch <XYZAC>\n"); break;
      default: rtapi_print_msg(RTAPI_MSG_INFO,"xyzac-switchkins: kinematicsSwitch <Trivkins>\n");
    }

    return 0; //0==> no error
}

static
int trivKinematicsForward(const double *joints,
              EmcPose * pos,
              const KINEMATICS_FORWARD_FLAGS * fflags,
              KINEMATICS_INVERSE_FLAGS * iflags)
{
    int i;

    for(i = 0; i < EMCMOT_MAX_JOINTS; i++) {
        switch(data->joints[i]) {
            case 0: SET(tran.x); break;
            case 1: SET(tran.y); break;
            case 2: SET(tran.z); break;
            case 3: SET(a); break;
            case 4: SET(c); break;
            case 5: SET(b); break;
            case 6: SET(u); break;
            case 7: SET(v); break;
            case 8: SET(w); break;
        }
    }

    return 0;
}


static
int xyzacKinematicsForward(const double *joints,
              EmcPose * pos,
              const KINEMATICS_FORWARD_FLAGS * fflags,
              KINEMATICS_INVERSE_FLAGS * iflags)
{

		double		x_rot_point=*(haldata->x_rot_point);
		double		y_rot_point=*(haldata->y_rot_point);
		double		z_rot_point=*(haldata->z_rot_point);
    double    dt = *(haldata->tool_offset);
    double    dy = *(haldata->y_offset);
    double    dz = *(haldata->z_offset);
    double a_rad = joints[JA]*TO_RAD;
    double c_rad = joints[JC]*TO_RAD;

    dz = dz + dt;


    pos->tran.x = + cos(c_rad)              * (joints[JX] - x_rot_point)
                  + sin(c_rad) * cos(a_rad) * (joints[JY] - dy - y_rot_point)
                  + sin(c_rad) * sin(a_rad) * (joints[JZ] - dz - z_rot_point )
                  + sin(c_rad) * dy
									+ x_rot_point;

    pos->tran.y = - sin(c_rad)              * (joints[JX] - x_rot_point )
                  + cos(c_rad) * cos(a_rad) * (joints[JY] - dy - y_rot_point)
                  + cos(c_rad) * sin(a_rad) * (joints[JZ] - dz - z_rot_point)
                  + cos(c_rad) * dy
									+ y_rot_point;

    pos->tran.z = + 0
                  - sin(a_rad) * (joints[JY] - dy - y_rot_point)
                  + cos(a_rad) * (joints[JZ] - dz - z_rot_point)
                  + dz
									+ z_rot_point;

    pos->a = joints[JA];
    pos->c = joints[JC];

    pos->b = 0;
    pos->w = 0;
    pos->u = 0;
    pos->v = 0;
	
    return 0;
}


int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{
    switch (data->switchkins_type) {
       case 1: return xyzacKinematicsForward( joints, pos, fflags, iflags );break;
      default: return trivKinematicsForward(  joints, pos, fflags, iflags );
    }

    return 0;
}

static
int trivKinematicsInverse(const EmcPose * pos,
              double *joints,
              const KINEMATICS_INVERSE_FLAGS * iflags,
              KINEMATICS_FORWARD_FLAGS * fflags)
{
    int i;
    for(i = 0; i < EMCMOT_MAX_JOINTS; i++) {
        switch(data->joints[i]) {
            case 0: joints[i] = pos->tran.x; break;
            case 1: joints[i] = pos->tran.y; break;
            case 2: joints[i] = pos->tran.z; break;
            case 3: joints[i] = pos->a; break;
            case 4: joints[i] = pos->c; break;
            case 5: joints[i] = pos->b; break;
            case 6: joints[i] = pos->u; break;
            case 7: joints[i] = pos->v; break;
            case 8: joints[i] = pos->w; break;
        }
    }

    return 0;
}

static
int xyzacKinematicsInverse(const EmcPose * pos,
              double *joints,
              const KINEMATICS_INVERSE_FLAGS * iflags,
              KINEMATICS_FORWARD_FLAGS * fflags)
{
		double		x_rot_point=*(haldata->x_rot_point);
		double		y_rot_point=*(haldata->y_rot_point);
		double		z_rot_point=*(haldata->z_rot_point);
    double    dy = *(haldata->y_offset);
    double    dz = *(haldata->z_offset);
    double    dt = *(haldata->tool_offset);
    double a_rad = pos->a*TO_RAD;
    double c_rad = pos->c*TO_RAD;

    dz = dz + dt;

    joints[JX] = + cos(c_rad)              * (pos->tran.x - x_rot_point)
                 - sin(c_rad)              * (pos->tran.y - y_rot_point)
								 + x_rot_point;

    joints[JY] = + sin(c_rad) * cos(a_rad) * (pos->tran.x - x_rot_point)
                 + cos(c_rad) * cos(a_rad) * (pos->tran.y - y_rot_point)
                 -              sin(a_rad) * (pos->tran.z - z_rot_point)
                 -              cos(a_rad) * dy
                 +              sin(a_rad) * dz
								 + dy
								 + y_rot_point;

    joints[JZ] = + sin(c_rad) * sin(a_rad) * (pos->tran.x - x_rot_point)
                 + cos(c_rad) * sin(a_rad) * (pos->tran.y - y_rot_point)
                 +              cos(a_rad) * (pos->tran.z - z_rot_point)
                 -              sin(a_rad) * dy
                 -              cos(a_rad) * dz
                 + dz
								 + z_rot_point;


    joints[JA] = pos->a;
    joints[JC] = pos->c;

    return 0;
}


int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{
    switch (data->switchkins_type) {
       case 1: return xyzacKinematicsInverse( pos, joints, iflags, fflags );break;
      default: return trivKinematicsInverse(  pos, joints, iflags, fflags );
    }

    return 0;
}

/* implemented for these kinematics as giving joints preference */
int kinematicsHome(EmcPose * world,
		   double *joint,
		   KINEMATICS_FORWARD_FLAGS * fflags,
		   KINEMATICS_INVERSE_FLAGS * iflags)
{
    *fflags = 0;
    *iflags = 0;

    return kinematicsForward(joint, world, fflags, iflags);
}

static KINEMATICS_TYPE ktype = -1;

KINEMATICS_TYPE kinematicsType()
{
    return ktype;
}

static char *coordinates = "XYZABCUVW";
RTAPI_MP_STRING(coordinates, "Existing Axes");

static char *kinstype = "1"; // use KINEMATICS_IDENTITY
RTAPI_MP_STRING(kinstype, "Kinematics Type (Identity,Both)");

EXPORT_SYMBOL(kinematicsSwitchable);
EXPORT_SYMBOL(kinematicsSwitch);
EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

static int next_axis_number(void) {
    while(*coordinates) {
	switch(*coordinates) {
	    case 'x': case 'X': coordinates++; return 0;
	    case 'y': case 'Y': coordinates++; return 1;
	    case 'z': case 'Z': coordinates++; return 2;
	    case 'a': case 'A': coordinates++; return 3;
	    case 'b': case 'B': coordinates++; return 4;
	    case 'c': case 'C': coordinates++; return 5;
	    case 'u': case 'U': coordinates++; return 6;
	    case 'v': case 'V': coordinates++; return 7;
	    case 'w': case 'W': coordinates++; return 8;
	    case ' ': case '\t': coordinates++; continue;
	}
	rtapi_print_msg(RTAPI_MSG_ERR,
		"xyzac-switchkins: ERROR: Invalid character '%c' in coordinates\n",
		*coordinates);
		return -1;
    }
    return -1;
}

int comp_id;
int rtapi_app_main(void) {
    int i;
    int res = 0;
    comp_id = hal_init("xyzac-switchkins");
    if(comp_id < 0) return comp_id;

    data = hal_malloc(sizeof(struct data));

    for(i=0; i<EMCMOT_MAX_JOINTS; i++) {
    	data->joints[i] = next_axis_number();
    }
    data->switchkins_type = 0; // startup default
    switch (*kinstype) {
      case 'b': case 'B': ktype = KINEMATICS_BOTH;         break;
      case 'f': case 'F': ktype = KINEMATICS_FORWARD_ONLY; break;
      case 'i': case 'I': ktype = KINEMATICS_INVERSE_ONLY; break;
      case '1': default:  ktype = KINEMATICS_IDENTITY;
    }

    haldata = hal_malloc(sizeof(struct haldata));

		if((res = hal_pin_float_new("xyzac-switchkins.x-rot-point",
              HAL_IO, &(haldata->x_rot_point), comp_id)) < 0) goto error;
		if((res = hal_pin_float_new("xyzac-switchkins.y-rot-point",
              HAL_IO, &(haldata->y_rot_point), comp_id)) < 0) goto error;
		if((res = hal_pin_float_new("xyzac-switchkins.z-rot-point",
              HAL_IO, &(haldata->z_rot_point), comp_id)) < 0) goto error;

    if((res = hal_pin_float_new("xyzac-switchkins.y-offset",
              HAL_IO, &(haldata->y_offset), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("xyzac-switchkins.z-offset",
              HAL_IO, &(haldata->z_offset), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("xyzac-switchkins.tool-offset",
              HAL_IN, &(haldata->tool_offset), comp_id)) < 0) goto error;


    rtapi_print_msg(RTAPI_MSG_ERR, "xyzac-switchkins: kinstype <%d>\n", ktype);
    hal_ready(comp_id);
    return 0;

error:
    hal_exit(comp_id);
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
