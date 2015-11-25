lib_LTLIBRARIES =	libpwr_rapldev.la \
			libpwr_xtpmdev.la \
			libpwr_wudev.la \
			libpwr_cpudev.la \
			libdummy_dev.la

# Power API Plugins
if HAVE_POWERGADGET
lib_LTLIBRARIES += libpwr_pgdev.la

libpwr_pgdev_la_SOURCES = pwr_dev.c pwr_pgdev.c
libpwr_pgdev_la_CFLAGS = -I$(top_srcdir)/src/pwr $(POWERGADGET_CFLAGS)
libpwr_pgdev_la_LDFLAGS = -version-info 1:0:1 $(POWERGADGET_LDFLAGS) $(POWERGADGET_LIBS)
endif

if HAVE_POWERINSIGHT
lib_LTLIBRARIES += libpwr_pidev.la

libpwr_pidev_la_SOURCES = pwr_dev.c pwr_pidev.c
libpwr_pidev_la_CFLAGS = -I$(top_srcdir)/src/pwr $(POWERINSIGHT_CFLAGS)
libpwr_pidev_la_LDFLAGS = -version-info 1:0:1 $(POWERINSIGHT_LDFLAGS) $(POWERINSIGHT_LIBS)
endif

if HAVE_WATTPROF
lib_LTLIBRARIES += libpwr_wpdev.la

libpwr_wpdev_la_SOURCES = pwr_dev.c pwr_wpdev.c
libpwr_wpdev_la_CFLAGS = -I$(top_srcdir)/src/pwr $(WATTPROF_CFLAGS)
libpwr_wpdev_la_LDFLAGS = -version-info 1:0:1 $(WATTPROF_LDFLAGS) $(WATTPROF_LIBS)
endif

libpwr_rapldev_la_SOURCES = pwr_dev.c pwr_rapldev.c
libpwr_rapldev_la_CFLAGS = -I$(top_srcdir)/src/pwr
libpwr_rapldev_la_LDFLAGS = -version-info 1:0:1

libpwr_xtpmdev_la_SOURCES = pwr_dev.c pwr_xtpmdev.c
libpwr_xtpmdev_la_CFLAGS = -I$(top_srcdir)/src/pwr
libpwr_xtpmdev_la_LDFLAGS = -version-info 1:0:1

libpwr_wudev_la_SOURCES = pwr_dev.c pwr_wudev.c
libpwr_wudev_la_CFLAGS = -I$(top_srcdir)/src/pwr
libpwr_wudev_la_LDFLAGS = -version-info 1:0:1

libpwr_cpudev_la_SOURCES = pwr_dev.c pwr_cpudev.c
libpwr_cpudev_la_CFLAGS = -I$(top_srcdir)/src/pwr
libpwr_cpudev_la_LDFLAGS = -version-info 1:0:1

libdummy_dev_la_SOURCES = pwr_dev.c dummy_dev.c
libdummy_dev_la_CFLAGS = -I$(top_srcdir)/src/pwr
libdummy_dev_la_LDFLAGS = -version-info 1:0:1


