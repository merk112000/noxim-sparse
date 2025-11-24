##### LIBRARIES CONFIGURATION #####

# Local installs in your project space
SYSTEMC        := /projects/befp/dm68/local/systemc-2.3.3
SYSTEMC_LIBDIR := $(SYSTEMC)/lib-linux64   # <-- change to lib-linux if that's your dir

YAML           := /projects/befp/dm68/local/yaml-cpp
YAML_LIBDIR    := $(YAML)/lib64              # where libyaml-cpp.so / .a lives


##### DIRECTORIES #####

SRCDIR  := ../src
SUBDIRS := $(filter %/,$(wildcard $(SRCDIR)/*/))
OBJDIR  := ./build

SRCS := $(wildcard $(SRCDIR)/*.cpp) $(wildcard $(addsuffix *.cpp,$(SUBDIRS)))
OBJS := $(subst $(SRCDIR),$(OBJDIR),$(SRCS:.cpp=.o))

MODULE := noxim


##### COMPILER OPTIONS #####

CXX      := g++
OPT      := -O3
# to enable logs with LOG macro, uncomment "-g -DDEBUG" in the line below
# and recompile everything (make clean)
#DEBUG    := -g -DDEBUG
OTHER    := -Wall -DSC_NO_WRITE_CHECK --std=c++11 # -Wno-deprecated
CXXFLAGS := $(OPT) $(OTHER) $(DEBUG)

# Include dirs: source, SystemC, yaml-cpp
INCDIR := -I$(SRCDIR) -isystem $(SYSTEMC)/include -I$(YAML)/include

# Library search paths: local src (if any libs there), SystemC, yaml-cpp
LIBDIR := -L$(SRCDIR) -L$(SYSTEMC_LIBDIR) -L$(YAML_LIBDIR)

# Actual libraries to link
LIBS := -lsystemc -lm -lyaml-cpp

SPACE := $(subst ,, )
VPATH := $(SRCDIR):$(subst $(SPACE),:,$(SUBDIRS))


##### RULES #####

$(MODULE): $(OBJS)
	$(CXX) $(LIBDIR) $^ $(LIBS) -o $@ 2>&1 | c++filt

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCDIR) -c $< -o $@

clean:
	rm -f $(OBJS) $(MODULE) *~

depend:
	makedepend $(SRCS) -Y -f- 2>/dev/null | sed 's_$(SRCDIR)_$(OBJDIR)_' > Makefile.deps

-include Makefile.deps
