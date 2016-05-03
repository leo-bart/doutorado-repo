# Name of the executable
Target = bin/main
exec_name = $(BINDIR)/wagonmassvar

# MBSim installation directory
mbsimdir = /home/leonardo/Software/mbsim

# Defining sources
SRCEXT = cpp
sources = $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))

# Custom includes
incl = -I include

# Directory paths
BINDIR = bin
BUILDDIR = build
SRCDIR = src

# Do not edit the following lines
CXX = g++
libsources = 
objects = $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(sources:.$(SRCEXT)=.o))
CPPFLAGS= -m32 -g3 -Wall -Werror -Wno-unknown-pragmas -fopenmp \
`$(mbsimdir)/bin/mbsim-config --cflags`
	#CPPFLAGS= -m32 -g3 -Wall -Werror -Wno-unknown-pragmas `pkg-config --cflags mbsim`
$(Target) : $(objects)
	@echo " Linking..."
	$(CXX) $^ -o $(exec_name) `$(mbsimdir)/bin/mbsim-config --libs` -lpthread -lm

# Build objects
$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@echo " Building..."
	@mkdir -p $(BUILDDIR)
	$(CXX) $(CPPFLAGS) $(incl) -c -o $@ $<
	
%.d: %.cc
	set -e; $(CXX) -MM $(CPPFLAGS) $< \
	  | sed 's/\(.*\)\.o[ :]*/\1.o \1.d : \g' > $@; \
	  [ -s $@ ] || rm -f $@
	
include $(sources:.cc:.d)
	
.PHONY : clean
clean :
	@echo " Cleaning..."
	$(RM) -r $(BUILDDIR) $(exec_name)
