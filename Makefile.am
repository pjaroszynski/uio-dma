SUBDIRS = kmod ulib tests

pkgconfdir = $(libdir)/pkgconfig
pkgconf_DATA = uio-dma.pc

topsrc   = @abs_top_srcdir@
rpmbuild = $(topsrc)/rpm-build
rpmdir   = $(top_srcdir)/RPMS
instdir  = $(top_srcdir)/usr/sbin

DISTCLEANFILES = autom4te.cache

MAINTAINERCLEANFILES = Makefile.in aclocal.m4 configure \
                       config.log config.status         \
                       config.sub depcomp install-sh    \
                       ltmain.sh missing config.guess   \
                       mkinstalldirs compile rpm.spec   \
		       Doxyfile

EXTRA_DIST = ulib.spec.in kmod.spec.in uio-dma.pc.in Doxyfile.in \
	     VERSION ChangeLog
rpm:
	rm -rf $(rpmbuild)
	mkdir -p $(rpmdir)
	make DESTDIR=$(rpmbuild) install
	rpmbuild --buildroot=$(rpmbuild)     \
		--define='_rpmdir $(rpmdir)' \
		--define='_unpackaged_files_terminate_build 0' \
		-bb uio-dma.spec
	rm -rf $(rpmbuild)

dox:
	doxygen

real-clean: maintainer-clean
	find . -name Makefile.in | xargs rm
	rm -rf include
	rm -rf $(rpmbuild) $(rpmdir)
