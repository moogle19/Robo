#-----------------------------------------------------------------------------
# Top-level Makefile for ANN.
#
# ANN: Approximate Nearest Neighbors
# Version: 0.1 (Beta release)
#-----------------------------------------------------------------------------
# Copyright (c) 1997 University of Maryland and Sunil Arya and David Mount
# All Rights Reserved.
# 
# This software and related documentation is part of the 
# Approximate Nearest Neighbor Library (ANN).
# 
# Permission to use, copy, and distribute this software and its 
# documentation is hereby granted free of charge, provided that 
# (1) it is not a component of a commercial product, and 
# (2) this notice appears in all copies of the software and
#     related documentation. 
# 
# The University of Maryland and the authors make no representations
# about the suitability or fitness of this software for any purpose.
# It is provided "as is" without express or implied warranty.
#-----------------------------------------------------------------------------
# Revision 0.1  09/06/97
#	Initial release
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# default: list the options
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# main make entry point
#-----------------------------------------------------------------------------
default:
	@cd src ; $(MAKE) 
	@cd test ; $(MAKE)
	@cd tools ; $(MAKE)
	@cd samples ; $(MAKE)

#-----------------------------------------------------------------------------
# Remove .o files and core files
#-----------------------------------------------------------------------------
clean:
	cd src ; $(MAKE) clean
	cd test ; $(MAKE) clean
	cd tools ; $(MAKE) clean
	cd samples ; $(MAKE) clean

#-----------------------------------------------------------------------------
# Remove everthing that can be remade
#-----------------------------------------------------------------------------
realclean: 
	-rm -f lib/*.a lib/*.so
	cd src ; $(MAKE) realclean
	cd test ; $(MAKE) realclean
	cd tools ; $(MAKE) realclean
	cd samples ; $(MAKE) realclean