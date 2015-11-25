
AC_DEFUN([PWRAPI_CHECK_WATTPROF], [
	AC_ARG_WITH([wattprof], [AS_HELP_STRING([--with-wattprof@<:@=DIR@:>@], [Use the wattprof binding in directory specified (DIR).])])
	pwrapi_check_wattprof_happy="yes"

	CPPFLAGS_saved="$CPPFLAGS"
  	LDFLAGS_saved="$LDFLAGS"
	LIBS_saved="$LIBS"
	WATTPROF_LIBS=""

	AS_IF([test -z "$with_wattprof"],
		[WATTPROF_CPPFLAGS=
                 WATTPROF_LDFLAGS=
                 WATTPROF_LIBS="-lrnet_host_pm_api -lpthread -lrt -lm"
                 LIBS="$LIBS $WATTPROF_LIBS"],
		[	AS_IF([test "x$with_wattprof" = "xyes"],
				[WATTPROF_CPPFLAGS=
           		 WATTPROF_LDFLAGS=
           		 WATTPROF_LIBS="-lrnet_host_pm_api -lpthread -lrt -lm"
           		 LIBS="$LIBS $WATTPROF_LIBS"],
				[WATTPROF_CPPFLAGS="-I$with_wattprof/include"
                 CPPFLAGS="$WATTPROF_CPPFLAGS $CPPFLAGS"
                 WATTPROF_LDFLAGS="-L$with_wattprof/lib"
                 LDFLAGS="$WATTPROF_LDFLAGS $LDFLAGS"
                 WATTPROF_LIBS="-lrnet_host_pm_api -lpthread -lrt -lm"
                 LIBS="$LIBS $WATTPROF_LIBS"]
		)]
	)

	AC_LANG_SAVE
	AC_LANG_CPLUSPLUS

	AC_CHECK_HEADERS([rnet_pm_api.h], [], [pwrapi_check_wattprof_happy="no"])
	AC_LINK_IFELSE([AC_LANG_PROGRAM([], [
			int a;
		])], [pwrapi_check_wattprof_lib_happy="yes"],
		[pwrapi_check_wattprof_lib_happy="no"])

	AS_IF([test "x$pwrapi_check_wattprof_lib_happy" = "xno"],
		[pwrapi_check_wattprof_happy="no"])

	AC_LANG_RESTORE

	CPPFLAGS="$CPPFLAGS_saved"
	LDFLAGS="$LDFLAGS_saved"
	LIBS="$LIBS_saved"

	AC_SUBST([WATTPROF_CPPFLAGS])
	AC_SUBST([WATTPROF_LDFLAGS])
	AC_SUBST([WATTPROF_LIBS])

	AM_CONDITIONAL([HAVE_WATTPROF], [test "x$pwrapi_check_wattprof_happy" = "xyes"])
	AS_IF([test "x$pwrapi_check_wattprof_happy" = "xyes"], 
		[AC_DEFINE([HAVE_WATTPROF], [1], [Set to 1 if wattprof is found during configuration])])

	AS_IF([test "x$pwrapi_check_wattprof_happy" = "xyes"], [$1], [$2])
])
