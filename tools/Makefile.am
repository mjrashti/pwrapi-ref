bin_PROGRAMS = pwrapi pwrgrp pwrgen pwrdmp

# Power API Tools
pwrapi_SOURCES = pwrapi.c
pwrapi_CFLAGS = -I$(top_srcdir)/src/pwr
pwrapi_LDADD = $(top_builddir)/src/pwr/libpwr.la

pwrgrp_SOURCES = pwrgrp.c
pwrgrp_CFLAGS = -I$(top_srcdir)/src/pwr
pwrgrp_LDADD = $(top_builddir)/src/pwr/libpwr.la

pwrgen_SOURCES = pwrgen.cc
pwrgen_CPPFLAGS = -I$(top_srcdir)/src/tinyxml2
pwrgen_LDADD = $(top_builddir)/src/tinyxml2/libtinyxml2.la

pwrdmp_SOURCES = pwrdmp.c
pwrdmp_CFLAGS = -I$(top_srcdir)/src/pwr
pwrdmp_LDADD = $(top_builddir)/src/pwr/libpwr.la

man_MANS = pwrapi.8 pwrgrp.8 pwrgen.8 pwrdmp.8

if HAVE_XMLRPC
bin_PROGRAMS += pwrsrv

pwrsrv_SOURCES = pwrsrv.cpp
pwrsrv_CPPFLAGS = $(XMLRPC_CPPFLAGS) -I$(top_srcdir)/src/pwr
pwrsrv_LDFLAGS = $(XMLRPC_LDFLAGS)
pwrsrv_LDADD = $(XMLRPC_LIBS) $(top_builddir)/src/pwr/libpwr.la

man_MANS += pwrsrv.8
endif

configdir = $(docdir)/config
config_DATA = 	config/shepard-platform.xml \
		config/shepard-node.xml \
		config/cooper-platform.xml \
		config/cooper-node.xml \
		config/teller-platform.xml \
		config/teller-node.xml

EXTRA_DIST = $(config_DATA)
