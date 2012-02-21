/********************************************************************
 * Description: interp_remap.cc
 *
 *  Remapping support
 *
 * Author: Michael Haberler
 * License: GPL Version 2
 * System: Linux
 *
 * Copyright (c) 2011 All rights reserved.
 *
 ********************************************************************/
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "rs274ngc.hh"
#include "rs274ngc_return.hh"
#include "rs274ngc_interp.hh"
#include "interp_internal.hh"

namespace bp = boost::python;


bool Interp::has_user_mcode(setup_pointer settings,block_pointer block)
{
    unsigned i;
    for(i = 0; i < sizeof(block->m_modes)/sizeof(int); i++) {
	if (block->m_modes[i] == -1)
	    continue;
	if (M_REMAPPABLE(block->m_modes[i]) &&
	    settings->m_remapped[block->m_modes[i]])
	    return true;
    }
    return false;
}

bool Interp::remap_in_progress(const char *code)
{
    remap_pointer rp = remapping(code);
    if (rp == NULL)
	return false;
    for (int i = _setup.remap_level; i > 0; i--) {
	if (_setup.blocks[i].executing_remap == rp) {
	    // printf("---------------- remap_in_progress(%s): TRUE level=%d\n",code,i);
	    return true;
	}
    }
    // printf("---------------- remap_in_progress(%s): FALSE\n",code);
    return false;
}


int Interp::convert_remapped_code(block_pointer block,
				  setup_pointer settings,
				  int phase,
				  char letter,
				  int number)
{
    remap_pointer remap;
    char key[2];
    int status;
    block_pointer cblock;
    bp::list plist;
    char cmd[LINELEN];

    if (number == -1)
	logRemap("convert_remapped_code '%c'", letter);
    else
	logRemap("convert_remapped_code '%c%d'", letter, number);

    switch (toupper(letter)) {
    case 'M':
	remap = settings->m_remapped[number];
	break;
    case 'G':
	remap = settings->g_remapped[number];
	break;
    default:
	key[0] = letter;
	key[1] = '\0';
	remap = remapping((const char *)key);
    }
    CHKS((remap == NULL), "BUG: convert_remapped_code: no remapping");

    // remapped handlers may use Python code to
    // setup environment before, and finish work after doing theirs.
    // That's what prolog and epilog functions are for.
    // These are described in the remap descriptor as read from ini.

    // Since a remap is always executed in the context of a controlling block,
    // this block now contains fields which hold dynamic remap information, like
    // the breadcrumbs execution trail.
    // Some of these fields are initialized here -
    // conceptually the block stack is also a 'remap frame stack'.

    // the O_call code will pick up the static descriptor and
    // dynamic information through the block and call any prolog
    // function before passing control to the actual handler procedure.

    // On the corresponding O_endsub/O_return, any epilog function
    // will be executed, doing any work not doable in an NGC file.

    // Note that even Python-remapped execution is pulled through the
    // oword mechanism - so no duplication of handler calling code
    // is needed.

    snprintf(cmd, sizeof(cmd),"O <%s> call ", REMAP_FUNC(remap));

    // the controlling block holds all dynamic remap information.
    cblock = &CONTROLLING_BLOCK(*settings);
    cblock->executing_remap = remap; // the current descriptor
    cblock->param_cnt = 0;

    if (remap->argspec && (strchr(remap->argspec, '@') != NULL)) {
    	// append a positional argument list instead of local variables
    	// if user specified '@'
	// named local params are dealt with in execute_call() when 
	// the new call frame is fully established
    	CHP(add_parameters(settings, cblock, &cmd[strlen(cmd)]));
    }

    if ((_setup.debugmask & EMC_DEBUG_REMAP) &&
	(_setup.loggingLevel > 2)) {
	logRemap("convert_remapped_code(%s)", cmd);
    }

    // good to go, pass to o-word call handling mechanism
    status = read(cmd);
    block_pointer eblock = &EXECUTING_BLOCK(*settings);
    eblock->call_type = CT_REMAP; 
    CHKS(status != INTERP_OK,
	 "convert_remapped_code: inital read returned %s",
	 interp_status(status));
    return(- phase);
}


// add_parameters - a built-in prolog function
//
// handles argspec and extracts required and optional items from the
// controlling block.
//
// if preparing for an NGC file, add local variables to
// the current oword subroutine call frame
//
// if posargs == NULL:
//      add the named parameters as local variables to  the current call frame
// if posargs != NULL:
//      create a positional argument list as per argspec order
//      instead of adding local variables
//
// also, generate a kwargs style dictionary of required and optional items
// in case a Python prolog is called
//
// 1. add all requried and  present optional words.
// 2. error on missing but required words.
// 4. handle '>' as to require a positive feed.
// 5. handle '^' as to require a positive speed.
// 6. handle 'N' as to add the line number.
//
// return INTERP_ERROR and propagate appropriate message if any errors so far
// else return INTERP_OK
//
// handling '@' (positional params) is dealt with in the calling procedure

int Interp::add_parameters(setup_pointer settings,
			   block_pointer cblock,
			   char *posarglist)
{
    const char *s,*argspec, *code;
    block_pointer block;
    char missing[30],optional[30],required[30];
    char *m = missing;
    char *o = optional;
    char *r = required;
    char msg[LINELEN], tail[LINELEN];
    bool errored = false;
    remap_pointer rptr = cblock->executing_remap;
    context_pointer active_frame = &settings->sub_context[settings->call_level];

    if (!rptr) {
	ERS("BUG: add_parameters: remap_frame: executing_remap == NULL ");
    }
    code = rptr->name;

    // if any Python handlers are present, create a kwargs dict
    bool pydict = rptr->remap_py || rptr->prolog_func || rptr->epilog_func;

    memset(missing,0,sizeof(missing));
    memset(optional,0,sizeof(optional));
    memset(required,0,sizeof(required));
    memset(msg,0,sizeof(msg));
    memset(tail,0,sizeof(tail));

    s = argspec = rptr->argspec;
    CHKS((argspec == NULL),"BUG: add_parameters: argspec = NULL");

    while (*s) {
	if (isupper(*s) && !strchr(required,*s)) *r++ = tolower(*s);
	if (islower(*s) && !strchr(optional,*s)) *o++ = *s;
	if (strchr(">^Nn",*s) && !strchr(required,*s)) *r++ = *s;
	s++;
    }
    o = optional;
    block = &CONTROLLING_BLOCK((*settings));

    logNP("add_parameters code=%s argspec=%s call_level=%d r=%s o=%s pydict=%d\n",
	    code,argspec,settings->call_level,required,optional,pydict);

#define STORE(name,value)						\
    if (pydict) {							\
	try {								\
	    active_frame->kwargs[name] = value;				\
        }								\
        catch (bp::error_already_set) {					\
	    PyErr_Print();						\
	    PyErr_Clear();						\
	    ERS("add_parameters: cant add '%s' to args",name);		\
	}								\
    }									\
    if (posarglist) {							\
	char actual[LINELEN];						\
	snprintf(actual, sizeof(actual),"[%.4lf]", value);		\
	strcat(posarglist, actual);					\
	cblock->param_cnt++;						\
    } else {								\
	add_named_param(name,0);					\
	store_named_param(settings,name,value,0);			\
    }


#define PARAM(spec,name,flag,value) 	                    	\
    if ((flag)) { /* present */	                    	        \
	/* required or optional */ 	                    	\
	if (strchr(required,spec) || strchr(optional,spec)) {	\
	    STORE(name,value);					\
	}							\
    } else {							\
	if (strchr(required,spec)) { /* missing */		\
	    *m++ = spec;					\
	    errored = true;					\
	}							\
    }

    s =  rptr->argspec;
    // step through argspec in order so positional args are built
    // in the correct order
    while (*s) {
	switch (tolower(*s)) {
	case 'a' : PARAM('a',"a",block->a_flag,block->a_number); break;
	case 'b' : PARAM('b',"b",block->b_flag,block->b_number); break;
	case 'c' : PARAM('c',"c",block->c_flag,block->c_number); break;
	case 'd' : PARAM('d',"d",block->d_flag,block->d_number_float); break;
	case 'e' : PARAM('e',"e",block->e_flag,block->e_number); break;
	case 'f' : PARAM('f',"f",block->f_flag,block->f_number); break;
	case 'h' : PARAM('h',"h",block->h_flag,(double) block->h_number); break;
	case 'i' : PARAM('i',"i",block->i_flag,block->i_number); break;
	case 'j' : PARAM('j',"j",block->j_flag,block->j_number); break;
	case 'k' : PARAM('k',"k",block->k_flag,block->k_number); break;
	case 'l' : PARAM('l',"l",block->l_flag,(double) block->l_number); break;
	case 'p' : PARAM('p',"p",block->p_flag,block->p_number); break;
	case 'q' : PARAM('q',"q",block->q_flag,block->q_number); break;
	case 'r' : PARAM('r',"r",block->r_flag,block->r_number); break;
	case 's' : PARAM('s',"s",block->s_flag,block->s_number); break;
	case 't' : PARAM('t',"t",block->t_flag, (double) block->t_number); break;
	case 'u' : PARAM('u',"u",block->u_flag,block->u_number); break;
	case 'v' : PARAM('v',"v",block->v_flag,block->v_number); break;
	case 'w' : PARAM('w',"w",block->w_flag,block->w_number); break;
	case 'x' : PARAM('x',"x",block->x_flag,block->x_number); break;
	case 'y' : PARAM('y',"y",block->y_flag,block->y_number); break;
	case 'z' : PARAM('z',"z",block->z_flag,block->z_number); break;
	case '-' : break; // ignore - backwards compatibility
	default: ;
	}
	s++;
    }

    s = missing;
    if (*s) {
	strcat(tail," missing: ");
    }
    while (*s) {
	errored = true;
	char c  = toupper(*s);
	strncat(tail,&c,1);
	if (*(s+1)) strcat(tail,",");
	s++;
    }
    // special cases:
    // N...add line number
    if (strchr(required,'n') || strchr(required,'N')) {
	STORE("n",(double) block->line_number);
    }

    // >...require positive feed
    if (strchr(required,'>')) {
	if (settings->feed_rate > 0.0) {
	    STORE("f",settings->feed_rate);
	} else {
	    strcat(tail,"F>0,");
	    errored = true;
	}
    }
    // ^...require positive speed
    if (strchr(required,'^')) {
	if (settings->speed > 0.0) {
	    STORE("s",settings->speed);
	} else {
	    strcat(tail,"S>0,");
	    errored = true;
	}
    }

    if (errored) {
	ERS("user-defined %s:%s",
	    code, tail);
    }
    return INTERP_OK;
}


// this looks up a remapping by unnormalized code (like G88.1)
remap_pointer Interp::remapping(const char *code)
{
    remap_iterator n = 	_setup.remaps.find(code);
    if (n !=  _setup.remaps.end())
	return &n->second;
    else
	return NULL;
}

// parse options of the form:
// REMAP= M420 modalgroup=6 argspec=pq prolog=setnamedvars ngc=m43.ngc epilog=ignore_retvalue
// REMAP= M421 modalgroup=6 argspec=- prolog=setnamedvars python=m43func epilog=ignore_retvalue

int Interp::parse_remap(const char *inistring, int lineno)
{

    char iniline[LINELEN];
    char *argv[MAX_REMAPOPTS];
    int   argc = 0;
    const char *code;
    remap r;
    bool errored = false;
    int g1 = 0, g2 = 0;
    int mcode = -1;
    int gcode = -1;
    char *s;

    memset((void *)&r, 0, sizeof(remap));
    r.modal_group = -1; // mark as unset, required param for m/g
    r.motion_code = INT_MIN;
    strcpy(iniline, inistring);
    // strip trailing comments
    if ((s = strchr(iniline, '#')) != NULL) {
	*s = '\0';
    }
    s = strtok((char *) iniline, " \t");

    while( s != NULL && argc < MAX_REMAPOPTS - 1) {
	argv[argc++] = s;
	s = strtok( NULL, " \t" );
    }
    if (argc == MAX_REMAPOPTS) {
	Error("parse_remap: too many arguments (max %d)", MAX_REMAPOPTS);
	goto fail;
    }
    argv[argc] = NULL;
    code = strstore(argv[0]);
    r.name = code;

    for (int i = 1; i < argc; i++) {
	int kwlen = 0;
	char *kw = argv[i];
	char *arg = strchr(argv[i],'=');
	if (arg != NULL) {
	    kwlen = arg - argv[i];
	    arg++;
	    if (!strlen(arg)) { // 'kw='
		Error("option '%s' - zero length value: %d:REMAP = %s",
		      kw,lineno,inistring);
		errored = true;
		continue;
	    }
	} else { // 'kw'
	    Error("option '%s' - missing '=<value>: %d:REMAP = %s",
		  kw,lineno,inistring);
	    errored = true;
	    continue;;
	}
	if (!strncasecmp(kw,"modalgroup",kwlen)) {
	    r.modal_group = atoi(arg);
	    continue;
	}
	if (!strncasecmp(kw,"argspec",kwlen)) {
	    size_t pos = strspn (arg,
				 "ABCDEFGHIJKLMNPQRSTUVWXYZabcdefghijklmnpqrstuvwxyz>^@");
	    if (pos != strlen(arg)) {
		Error("argspec: illegal word '%c' - %d:REMAP = %s",
		      arg[pos],lineno,inistring);
		errored = true;
		continue;
	    }
	    r.argspec = strstore(arg);
	    continue;
	}
	if (!strncasecmp(kw,"prolog",kwlen)) {
	    if (PYUSABLE) {
		r.prolog_func = strstore(arg);
	    } else {
		Error("Python plugin required for prolog=, but not available: %d:REMAP = %s",
		      lineno,inistring);
		errored = true;
		continue;
	    }
	    continue;
	}
	if (!strncasecmp(kw,"epilog",kwlen)) {
	    if (PYUSABLE) {
		r.epilog_func = strstore(arg);
	    } else {
		Error("Python plugin required for epilog=, but not available: %d:REMAP = %s",
		      lineno,inistring);
		errored = true;
		continue;
	    }
	    continue;
	}
	if (!strncasecmp(kw,"ngc",kwlen)) {
	    if (r.remap_py) {
		Error("cant remap to an ngc file and a Python function: -  %d:REMAP = %s",
		      lineno,inistring);
		errored = true;
		continue;
	    }
	    FILE *fp = find_ngc_file(&_setup,arg);
	    if (fp) {
		r.remap_ngc = strstore(arg);
		fclose(fp);
	    } else {
		Error("NGC file not found: ngc=%s - %d:REMAP = %s",
		      arg, lineno,inistring);
		errored = true;
	    }
	    continue;
	}
	if (!strncasecmp(kw,"python",kwlen)) {
	    if (r.remap_ngc ) {
		Error("cant remap to an ngc file and a Python function: -  %d:REMAP = %s",
		      lineno,inistring);
		errored = true;
		continue;
	    }
	    if (!PYUSABLE) {
		Error("Python plugin required for python=, but not available: %d:REMAP = %s",
		      lineno,inistring);
		errored = true;
		continue;
	    }
	    if (!is_pycallable(&_setup, REMAP_MODULE, arg)) {
		Error("'%s' is not a Python callable function - %d:REMAP = %s",
		      arg,lineno,inistring);
		errored = true;
		continue;
	    }
	    r.remap_py = strstore(arg);
	    continue;
	}
	Error("unrecognized option '%*s' in  %d:REMAP = %s",
	      kwlen,kw,lineno,inistring);
    }
    if (errored) {
	goto fail;
    }

    if (remapping(code)) {
	Error("code '%s' already remapped : %d:REMAP = %s",
	      code,lineno,inistring);
	goto fail;
    }

    // it is an error not to define a remap function to call.
    if ((r.remap_ngc == NULL) && (r.remap_py == NULL)) {
	Error("code '%s' - no remap function given, use either 'python=<function>' or 'ngc=<basename>' : %d:REMAP = %s",
	      code,lineno,inistring);
	goto fail;
    }

#define CHECK(bad, fmt, ...)			\
    do {					\
	if (bad) {				\
	    Log(fmt, ## __VA_ARGS__);		\
	    goto fail;				\
	}					\
    } while(0)

    switch (towlower(*code)) {

    case 't':
    case 's':
    case 'f':
	CHECK((strlen(code) > 1),"%d: %c remap - only single letter code allowed", lineno, *code);
	CHECK((r.modal_group != -1), "%d: %c remap - modal group setting ignored - fixed sequencing", lineno, *code);
	_setup.remaps[code] = r;
	break;

    case 'm':
	if (sscanf(code + 1, "%d", &mcode) == 1) {
	    _setup.remaps[code] = r;
	    _setup.m_remapped[mcode] = &_setup.remaps[code];
	} else {
	    Error("parsing M-code: expecting integer like 'M420', got '%s' : %d:REMAP = %s",
		  code,lineno,inistring);
	    goto fail;
	}
	if (r.modal_group == -1) {
	    Error("warning: code '%s' : no modalgroup=<int> given, using default group %d : %d:REMAP = %s",
		  code, MCODE_DEFAULT_MODAL_GROUP,lineno,inistring);
	    r.modal_group = MCODE_DEFAULT_MODAL_GROUP;
	}
	if (!M_MODE_OK(r.modal_group)) {
	    Error("error: code '%s' : invalid modalgroup=<int> given (currently valid: 5..10) : %d:REMAP = %s",
		  code,lineno,inistring);
	    goto fail;
	}
	break;
    case 'g':

	// code may be G88.1 or so - normalize to use 'G881' instead
	// (multiply by 10, no dots)
	if (sscanf(code + 1, "%d.%d", &g1, &g2) == 2) {
	    gcode = g1 * 10 + g2;
	}
	if (( gcode == -1) &&  (sscanf(code + 1, "%d", &gcode) != 1)) {
	    Error("code '%s' : cant parse G-code : %d:REMAP = %s",
		  code, lineno, inistring);
	    goto fail;
	}
	r.motion_code = gcode;
	if (r.modal_group == -1) {
	    Error("warning: code '%s' : no modalgroup=<int> given, using default group %d : %d:REMAP = %s",
		  code, GCODE_DEFAULT_MODAL_GROUP, lineno, inistring);
	    r.modal_group = GCODE_DEFAULT_MODAL_GROUP;
	    break;
	}
	if (!G_MODE_OK(r.modal_group)) {
	    Error("error: code '%s' : %s modalgroup=<int> given  : %d:REMAP = %s",
		  argv[0],
		  r.modal_group == -1 ? "no" : "invalid",
		  lineno,
		  inistring);
	    goto fail;
	}
	_setup.remaps[code] = r;
	_setup.g_remapped[gcode] = &_setup.remaps[code];
	break;

    default:
	// make sure the python plugin is in a usable state if needed
	if ((r.prolog_func || r.remap_py || r.epilog_func) &&
	    (!PYUSABLE))  {
	    fprintf(stderr, "fatal: REMAP requires the Python plugin, which did not initialize\n");
	    break;
	}
	Log("REMAP BUG=%s %d:REMAP = %s",
	    code,lineno,inistring);
    }
    return INTERP_OK;

 fail:
    return INTERP_ERROR;
}
